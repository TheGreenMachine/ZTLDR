# Camera Calibration Save Files

This folder is for storing camera calibration saves from PhotonVision.

Because PhotonVision identifies cameras based on the port they are attached to on the coprocessor,
if a camera is moved to a different port or coprocessor, or we swap in a replacement camera, the
calibration for the camera needs to be imported.

Using a calibration from a different camera will likely lead to inaccurate AprilTag pose
estimation, as each camera, even if it is the same model, has physical differences. This
calibration also depends on the focus of the camera.

Calibrations can be saved by clicking on the calibration under the Camera Calibration section of
the Camera tab in the PhotonVision UI and clicking EXPORT. The downloaded JSON should then be saved
to this folder. Make sure the name of the saved file clearly identifies the camera it is for and
the resolution of the calibration.

To put an existing calibration onto the coprocessor port, click IMPORT on the same screen and
select the correct calibration.
