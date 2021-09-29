import wx
import numpy as np

from cockpit.experiment import experiment, actionTable
from cockpit.gui import guiUtils
import cockpit.util.userConfig
import cockpit.util.logger
from cockpit import depot
import decimal

## Provided so the UI knows what to call this experiment.
EXPERIMENT_NAME = "CryoSIM"

## This class handles slm test experiment
class CryoSIM(experiment.Experiment):
    ## \param numAngles How many angles to perform -- sometimes we only want
    # to do 1 angle, for example.
    ## \param numPhases How many phases to perform -- sometimes we only want
    # to do 1 angle, for example.
    # \param slmHandler Optionally, both angle and phase can be handled by an
    #        SLM or similar pattern-generating device instead. Each handler
    #        (angle, phase, and slm) will be used if present.
    # \param rotorHandler Optionally, both angle and phase can be handled by an
    #        SLM or similar pattern-generating device instead. Each handler
    #        (angle, phase, and slm) will be used if present.
    def __init__(
        self, numAngles, numPhases, slmHandler=None, rotorHandler=None, *args, **kwargs
    ):

        print(f"#a {numAngles} #p {numPhases}")
        w = 488e-9
        self.sequence = []
        angle = 180
        phase = 360
        for i in range(numAngles):
            for j in range(numPhases):
                self.sequence.append((i * angle / numAngles, j * phase / numPhases, w))

        # Store the diffraction angle in MRC metadata
        self.slm = depot.getDeviceWithName("slm")
        self.slm.set_sequence(self.sequence)

        self.rotor = depot.getDeviceWithName("rotor")
        self.rotor.set_sequence(self.sequence)

        if self.slm:
            diffangle = self.slm.angle
            metadata = ": SLM diff_angle %.3f" % diffangle
        if "metadata" in kwargs:
            # Augment the existing string.
            kwargs["metadata"] += "; %s" % metadata
        else:
            kwargs["metadata"] = metadata
        print("metadata %s" % metadata)

        super().__init__(*args, **kwargs)
        self.numAngles = numAngles
        self.numPhases = numPhases
        self.numZSlices = int(np.ceil(self.zHeight / self.sliceHeight))
        if self.zHeight > 1e-6:
            # Non-2D experiment; tack on an extra image to hit the top of
            # the volume.
            self.numZSlices += 1
        self.slmHandler = slmHandler
        self.rotorHandler = rotorHandler

    # needed for the experiment
    def generateActions(self):
        table = actionTable.ActionTable()
        curTime = 0

        camsToReset = set()
        for camera in self.cameras:
            if not self.cameraToIsReady[camera]:
                camsToReset.add(camera)
        if camsToReset:
            curTime = self.resetCams(curTime, camsToReset, table)

        prevAltitude = None
        numZSlices = int(np.ceil(self.zHeight / self.sliceHeight))
        if self.zHeight > 1e-6:
            # Non-2D experiment; tack on an extra image to hit the top of
            # the volume.
            numZSlices += 1
        for zIndex in range(numZSlices):
            # Move to the next position, then wait for the stage to
            # stabilize.
            zTarget = self.zStart + self.sliceHeight * zIndex
            motionTime, stabilizationTime = 0, 0
            if prevAltitude is not None:
                motionTime, stabilizationTime = self.zPositioner.getMovementTime(
                    prevAltitude, zTarget
                )
            curTime += motionTime
            table.addAction(curTime, self.zPositioner, zTarget)
            curTime += stabilizationTime
            prevAltitude = zTarget

            for i in range(len(self.sequence)):
                if (
                    self.slmHandler is not None
                    and len(self.slmHandler.analogClients) > 0
                ):
                    for client in self.slmHandler.analogClients:
                        table.addAction(curTime, client, i)
                if (
                    self.rotorHandler is not None
                    and len(self.rotorHandler.analogClients) > 0
                ):
                    for client in self.rotorHandler.analogClients:
                        table.addAction(curTime, client, i)

                curTime = self.expose(curTime, self.cameras, table)
                # Wait a few ms for any necessary SLM triggers.
                curTime += decimal.Decimal("5")  # ms

            # Hold the Z motion flat during the exposure.
            table.addAction(curTime, self.zPositioner, zTarget)

        # Move back to the start so we're ready for the next rep.
        motionTime, stabilizationTime = self.zPositioner.getMovementTime(
            self.zHeight, 0
        )
        curTime += motionTime
        table.addAction(curTime, self.zPositioner, self.zStart)
        # Hold flat for the stabilization time, and any time needed for
        # the cameras to be ready. Only needed if we're doing multiple
        # reps, so we can proceed immediately to the next one.
        cameraReadyTime = 0
        if self.numReps > 1:
            for cameras, lightTimePairs in self.exposureSettings:
                for camera in cameras:
                    cameraReadyTime = max(
                        cameraReadyTime, self.getTimeWhenCameraCanExpose(table, camera)
                    )
        table.addAction(
            max(curTime + stabilizationTime, cameraReadyTime),
            self.zPositioner,
            self.zStart,
        )

        return table

    # needed for generateActions
    def expose(self, curTime, cameras, table):
        max_acq_time = 0
        last_ready = 0

        for camera in cameras:
            exposure = camera.getExposureTime()
            # getTimeBetweenExposures has to be read after set exposure (at run)
            # instead of at initialization
            waiting = camera.getTimeBetweenExposures()
            if max_acq_time < exposure + waiting:
                max_acq_time = decimal.Decimal(exposure + waiting)
            ready = self.getTimeWhenCameraCanExpose(table, camera)
            if last_ready < ready:
                last_ready = decimal.Decimal(ready)

        for light in self.lights:
            table.addAction(curTime, light, True)

        for camera in cameras:
            table.addAction(curTime, camera, True)
            self.cameraToImageCount[camera] += 1

        curTime += max_acq_time

        exposureEndTime = max(last_ready, curTime)
        return exposureEndTime


## A consistent name to use to refer to the class itself.
EXPERIMENT_CLASS = CryoSIM


class ExperimentUI(wx.Panel):
    # _CONFIG_KEY_SUFFIX = 'SIExperimentSettings'

    def __init__(self, parent, configKey):
        super().__init__(parent=parent)
        self.configKey = configKey
        self.settings = self.loadSettings()

        self.simArgs = {}
        sizer = wx.GridSizer(1, 2, 2, 2)
        options = [
            ('cryosimNangles',
             'Number of angles',
             'Number of divisions of the 360 angular range',
             guiUtils.INTVALIDATOR),
            ('cryosimNphases',
             'Number of phases',
             'Number of divisions of the 180 phase range',
             guiUtils.INTVALIDATOR),
        ]

        for key, label, helperString, validator in options:
            control = guiUtils.addLabeledInput(self, sizer, 
                label = label, defaultValue = self.settings[key],
                helperString = helperString)
            control.SetValidator(validator)
            self.simArgs[key] = control
        self.SetSizerAndFit(sizer)


    ## Given a parameters dict (parameter name to value) to hand to the
    # experiment instance, augment them with our special parameters.
    def augmentParams(self, params):
        self.saveSettings()
        numAngles = self.simArgs['cryosimNangles']
        params["numAngles"] = guiUtils.tryParseNum(numAngles)
        numPhases = self.simArgs['cryosimNphases']
        params["numPhases"] = guiUtils.tryParseNum(numPhases)
        params["slmHandler"] = depot.getHandler("slm", depot.EXECUTOR)
        params["rotorHandler"] = depot.getHandler("rotor", depot.EXECUTOR)
        return params

    ## Load the saved experiment settings, if any.
    def loadSettings(self):
        default = {
            'cryosimNangles': '3',
            'cryosimNphases': '5',
        }
        result = cockpit.util.userConfig.getValue(self.configKey + '_cryosim',
                                                  default=default)
        return result

    ## Generate a dict of our settings.
    def getSettingsDict(self):
        return {key: c.GetValue() for key, c in self.simArgs.items()}


    ## Save the current experiment settings to config.
    def saveSettings(self, settings = None):
        if settings is None:
            settings = self.getSettingsDict()
        cockpit.util.userConfig.setValue(
                self.configKey + '_cryosim',
                settings)
