list = GetArrayVariable("LINE_PATTERN")
if isArray(list) then
  if ubound(list) > 0 then

		' based on a line pattern in 8 x actual inches
		' of 10 17 31 32 10 = 100

    targetPixelHeight = (list(3)/10)
    targetSamples = 0

    ' calibrated for an Axis camera
    imageHeight = GetVariable("IMAGE_HEIGHT")
    cameraFieldOfView = 47.5
    targetHeight = 100.0

		' determine distance in 8 x inches
    totalDistance = (((targetHeight*imageHeight)/targetPixelHeight)/2)/_
      tan(((cameraFieldOfView*3.14159)/180.0)/2.0)

		' convert to ft (12 inch per ft * 8 inch multiplier) = 96
		totalDistance = CInt((totalDistance*100)/96)/100

		' save it for use in other modules
    SetVariable "Distance", totalDistance
		SetVariable "3882_TARGET_FOUND", 1.0
		SetVariable "3882_DELTA", 160 - list(1)


  end if
end if
