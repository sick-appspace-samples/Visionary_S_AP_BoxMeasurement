--[[----------------------------------------------------------------------------

  Application Name: Visionary_S_AP_BoxMeasurement

  Summary:
  To measure the width and length of boxes

  Description:
  The app uses the Shape3D.fitPlane() with LEASTSQUARES and RANSAC in combination

  Following steps are done for each image to calculate the dimensions of the box:
    1. Find connected flat regions in the image
    2. From the flat regions, retain only the ones which contain the center pixel - called box region
    3. Widen the box region by dilation and calculate its edges
    4. Find the largest connected region from the edge image
    5. Dilate and fill the holes in the identified region, get its borders
    6. In the point cloud, only retain the points that lie within the identified border and within the
       working range
    7. Rotate the point cloud to find the rotation angle where the point cloud is axis aligned
    8. From the point cloud of the box, find its bounding box and calculate the dimensions

  How to run:
  Start by running the app (F5) or debugging (F7+F10).
  Set a breakpoint on the first row inside the main function to debug step-by-step.
  See the results in the different image viewer on the DevicePage.

------------------------------------------------------------------------------]]
--Start of Global Scope---------------------------------------------------------
Log.setLevel("INFO")
-- Variables, constants, serves etc. should be declared here.

local zMapDecoration = View.ImageDecoration.create()
zMapDecoration:setRange(0, 1500)

local blueRoiDecoration = View.PixelRegionDecoration.create()
blueRoiDecoration:setColor(0, 0, 250, 100)
local redRoiDecoration = View.PixelRegionDecoration.create()
redRoiDecoration:setColor(250, 0, 0, 200)
local greenRoiDecoration = View.PixelRegionDecoration.create()
greenRoiDecoration:setColor(0, 250, 0, 200)

local shapeDecoration = View.ShapeDecoration.create()
shapeDecoration:setLineColor(255, 0, 0, 75)
shapeDecoration:setLineWidth(2.2)

local centerPointDecoration = View.ShapeDecoration.create()
centerPointDecoration:setLineColor(255, 0, 0, 255)
centerPointDecoration:setLineWidth(2.2)

local pointCloudDecoration = View.PointCloudDecoration.create()
pointCloudDecoration:setPointSize(1)

local camera = Image.Provider.Camera.create()
local camModel = camera:getInitialCameraModel()
local pointCloudConverter = Image.PointCloudConversion.PlanarDistance.create()
pointCloudConverter:setCameraModel(camModel)

local resultViewer = View.create("result-viewer")
local connectedRegionViewer = View.create("connected-region-viewer")
local connectedRegionEdgesViewer = View.create("connected-region-edges-viewer")
local box3DViewer = View.create("box-3d-viewer")

local MIN_REGION_SIZE = 2000

--End of Global Scope-----------------------------------------------------------

--Start of Function and Event Scope---------------------------------------------
local function main()
  -- write app code in local scope, using API
  resultViewer:present()
  connectedRegionViewer:present()
  connectedRegionEdgesViewer:present()
  box3DViewer:present()
  camera:stop()
  local config = camera.getDefaultConfig(camera)
  config:setMaps( { "z_u16", "image_rgba" } )
  config:setColorMappingMode("DISPARITY_ON_RGB")
  config:setIlluminationMode("SEQUENTIAL")
  config:enableIntIllum(true)
  config:setMinimumIdleTime(0)
  config:setAcquisitionMode("NORMAL") -- NORMAL, HDR, HQM
  config:setStereoIntegrationTime(2000)
  config:setStereoSecondIntegrationTime(100)
  config:setColorIntegrationTime(5000)
  config:enableDepthValidation(true)
  config:setDepthValidationLevel(8)

  -- for high res maps keep fps below 5 !
  -- web viewers performance is too bad
  config.setFramePeriod(config, 333000) -- micro seconds
  camera.setConfig(camera, config)
  camera:start()
end

--------------------------------------------------------------------------------

---@param pointCloud PointCloud
local function transformPlaneToXy(pointCloud)

  -- Get the parameters of the fitted plane from the point cloud
  local centroid = pointCloud:getCentroid()
  local points, _ = pointCloud:toPoints()
  local boxPlane = Shape3D.fitPlane(points, "TRIMMED", "ABSOLUTE")
  local nx, ny, nz, _ = boxPlane:getPlaneParameters()


  -- Build the transformation matrix from the fitted plane to tranform the plane
  -- to the xy plane on the point cloud

  -- !!! shortform !!!
  --   because we uses the unit vector from the plane
  --   and we are rotating to vector (0,0,1)
  local absNormPlane =  math.sqrt(nx*nx + ny*ny + nz*nz)
  local cosTheta = nz / absNormPlane
  local sinTheta = math.sqrt((nx*nx + ny*ny) / (nx*nx + ny*ny + nz*nz))
  local u1 = ny / math.sqrt(nx*nx + ny*ny)
  local u2 = (nx / math.sqrt(nx*nx + ny*ny)) * -1.0

  local translationMatrix = Matrix.createIdentity(4)
  translationMatrix:setValue(0, 3, -centroid:getX())
  translationMatrix:setValue(1, 3, -centroid:getY())
  translationMatrix:setValue(2, 3, -centroid:getZ())

  local rotationMatrix = Matrix.createIdentity(4)
  rotationMatrix:setValue(0, 0, cosTheta + u1 * u1 * (1- cosTheta))
  rotationMatrix:setValue(0, 1, u1 * u2 * (1- cosTheta))
  rotationMatrix:setValue(0, 2, u2 * sinTheta)
  rotationMatrix:setValue(1, 0, u1 * u2 * (1- cosTheta))
  rotationMatrix:setValue(1, 1, cosTheta + u2 * u2 * (1- cosTheta))
  rotationMatrix:setValue(1, 2, u1 * sinTheta * -1.0)
  rotationMatrix:setValue(2, 0, u2 * sinTheta * -1.0)
  rotationMatrix:setValue(2, 1, u1 * sinTheta)
  rotationMatrix:setValue(2, 2, cosTheta)

  local transformationMatrix = rotationMatrix:multiply(translationMatrix) -- first translate
  local transformObject = Transform.createFromMatrix3D(transformationMatrix)

  -- Apply the transormation to the point cloud
  pointCloud:transformInplace(transformObject)
end

--------------------------------------------------------------------------------

---@param image Image[]
---@param sensordata SensorData
local function handleOnNewImage(image)
  local startTime = DateTime.getTimestamp()
  local width = image[1]:getWidth()
  local height = image[1]:getHeight()
  local MAX_REGION_SIZE = width * height * 0.5

  local rgbImage = image[2]
  local grayImage = rgbImage:toGray()

  -------------------------------- FIND FLAT REGION AT CENTER --------------------------------

  -- First identify the flat regions that are connected in the image
  local flats = Image.getFlatRegion(image[1], 2 , 3, false)
  local connectedFlatRegions = flats:findConnected(MIN_REGION_SIZE, MAX_REGION_SIZE, 10)

  -- find the flat which includes the center pixel)
  local center = Point.create(width/2, height/2)
  local index = 0
  for i, v in ipairs(connectedFlatRegions) do
    if v:contains(center) then
      index = i
      break
    end
  end

  local boxRegion
  if index > 0 then
    -- widen the selected area a bit this ensures we have enough pixels
    -- from the rgb map for edge detection
    boxRegion = connectedFlatRegions[index]:dilate(3)
  else
    Log.info("ERROR: could not find connected region in center of image")
    resultViewer:clear()
    resultViewer:addImage(image[1], zMapDecoration, "zmap")
    resultViewer:addPixelRegion(flats, blueRoiDecoration, "flats", "zmap")
    resultViewer:addShape(Shape.createCircle(center, 3), centerPointDecoration, "centerpoint", "zmap")
    resultViewer:present()
    return -- early out
  end

  -------------------------------- CALCULATE BOX REGION FROM EDGE INFO --------------------------------

  -- calculate sobel edges in grayscale rgb, paint unused pixels black
  local edges = grayImage:sobelMagnitude(boxRegion)
  edges:fillRegionInplace(boxRegion:invert(edges), 255)

  -- select flat/dark pixels from edges map (based on rgb map)
  local prEdges = edges:threshold(0, 50, boxRegion)
  -- select biggest connected region
  -- !! chaotic labeled/printed boxes will not work !!
  local connected = prEdges:findConnected(MIN_REGION_SIZE, nil, 10)

  local prBox
  if #connected > 0 then
    prBox = connected[1]:dilate(3):getIntersection(boxRegion):fillHoles():getBorderRegion()
  else
    Log.info("ERROR: could not find biggest connected region from edge filter map")
    resultViewer:clear()
    resultViewer:addImage(image[1], zMapDecoration, "zmap")
    resultViewer:addShape(Shape.createCircle(center, 3), centerPointDecoration, "centerpoint", "zmap")
    resultViewer:present()
    return -- early out
  end

  -------------------------------- FILTER POINTS IN BOX REGION FROM POINT CLOUD --------------------------------

  -- pointcloud created from svs edp converter
  local boxPointCloud = pointCloudConverter:toPointCloud(image[1], image[1], prBox)

  -- filter measurement range, Recommended Working range: 400mm to 2600mm
  local filterPointCloud = PointCloud.RangeFilter.create()
  filterPointCloud:setZRange(400, 2600)
  boxPointCloud = filterPointCloud:filter(boxPointCloud)

  if boxPointCloud == nil or boxPointCloud:getSize() == 0 then
    return
  end
  local points, _ = boxPointCloud:toPoints()
  if points == nil then
    return
  end
  local boxPlane = Shape3D.fitPlane(points, "TRIMMED", "ABSOLUTE")
  if boxPlane == nil then
    return
  end

  -------------------------------- TRANFORM POINT CLOUD TO XY PLANE --------------------------------

  -- transform the plane onto the XY plane
  transformPlaneToXy(boxPointCloud)

  -- rotate around Z in steps and find the angle where the box is axis aligned
  local xSize = nil
  local temporaryPointCloud = PointCloud.create()
  local xMin, yMin, xMax, yMax
  local step = 3.14156 / 100
  local currentRotationAngle = step
  local rotationAngle = nil
  while currentRotationAngle < 3.14156 do
    temporaryPointCloud = boxPointCloud:rotateZ(currentRotationAngle)
    xMin, _, _, xMax, _, _ = temporaryPointCloud:getBounds()
    if xSize == nil then
      xSize =  xMax- xMin
      rotationAngle = currentRotationAngle
    else
      if xSize > (xMax - xMin) then
        xSize =  xMax- xMin
        rotationAngle = currentRotationAngle
      end
    end
    currentRotationAngle = currentRotationAngle + step
  end

  --  rotate the point cloud by the calculated angle
  boxPointCloud = boxPointCloud:rotateZ(rotationAngle)

  -- find the region around the box
  local aroundBox = boxRegion:dilate(51):getDifference(boxRegion):erode(9)

  -- fit the plane of the surface on which the box lies
  local environmentPointCloud = pointCloudConverter:toPointCloud(image[1], image[1], aroundBox)
  environmentPointCloud = filterPointCloud:filter(environmentPointCloud)
  points, _ = environmentPointCloud:toPoints()
  local environmentPlane = Shape3D.fitPlane(points, "TRIMMED", "ABSOLUTE")
  if environmentPlane == nil then
    return
  end

  -- calculate the normals to the plane of the surface and the box
  local _, _, _, boxNormalDist = boxPlane:getPlaneParameters()
  local _, _, _, envNormalDist = environmentPlane:getPlaneParameters()

  -- calculate the dimensions of the bounding box
  local bBox = boxPointCloud:getBoundingBox()
  xMin, yMin, _, xMax, yMax, _ = bBox:getBounds()
  local boxWidth = xMax-xMin
  local boxHeight = yMax-yMin
  local info = string.format("%.0fmm x %.0fmm (H: %.0fmm)", boxWidth, boxHeight, envNormalDist - boxNormalDist)


  -------------------------------- UPDATE VIEWERS --------------------------------
  resultViewer:clear()
  connectedRegionViewer:clear()
  connectedRegionEdgesViewer:clear()
  box3DViewer:clear()

  resultViewer:addImage(image[1], zMapDecoration, "zmap")
  resultViewer:addPixelRegion(boxRegion, blueRoiDecoration, "region", "zmap")
  resultViewer:addPixelRegion(prBox, redRoiDecoration, "region2", "zmap")
  resultViewer:addPixelRegion(aroundBox, greenRoiDecoration, nil, "zmap")
  resultViewer:addShape(Shape.createCircle(center, 3), centerPointDecoration, "centerpoint", "zmap")
  local textDecoration = View.TextDecoration.create()
  textDecoration:setPosition(width/2 - 60, height/2 - 20)
  textDecoration:setSize(20.0)
  textDecoration:setFontWeight("BOLD")
  textDecoration:setColor(255, 0, 0, 255)
  resultViewer:addText(info, textDecoration, "dim", "zmap")

  local connectedRegionViewerid =  connectedRegionViewer:addImage(rgbImage, nil)
  connectedRegionViewer:addPixelRegion(boxRegion, blueRoiDecoration, "notused", connectedRegionViewerid)
  connectedRegionViewer:addPixelRegion(prBox, redRoiDecoration, "region2", connectedRegionViewerid)

  local connectedRegionEdgesViewerid = connectedRegionEdgesViewer:addImage(edges)
  connectedRegionEdgesViewer:addPixelRegion(prBox, redRoiDecoration, "prBox", connectedRegionEdgesViewerid)

  -- workaround: merge both pointCloud as the 3D viewer
  -- can show only a single pointCloud
  --boxPointCloud:mergeInplace(environmentPointCloud)
  local box3DViewerid = box3DViewer:addPointCloud(boxPointCloud, pointCloudDecoration)
  box3DViewer:addShape(bBox, shapeDecoration, "l1", box3DViewerid)
  --box3DViewer:addShape(boxPlane, shapeDecoration, "l2", box3DViewerid)
  --box3DViewer:addShape(environmentPlane, shapeDecoration, "l3", box3DViewerid)

  local boxToVisualize = Shape3D.createBox(21,21,21)
  local box3DDecoration = View.ShapeDecoration.create()
  box3DDecoration:setLineColor(0, 0, 0, 255)
  box3DDecoration:setFillColor(255, 0, 200, 255)
  box3DViewer:addShape(boxToVisualize, box3DDecoration, "center", box3DViewerid)
  local lineX = Shape3D.createLineSegment(Point.create(0,0,0), Point.create(84,0,0))
  local lineY = Shape3D.createLineSegment(Point.create(0,0,0), Point.create(0,84,0))
  local lineZ = Shape3D.createLineSegment(Point.create(0,0,0), Point.create(0,0,84))
  box3DDecoration:setLineWidth(5)
  box3DDecoration:setLineColor(255, 0, 0)
  box3DViewer:addShape(lineX, box3DDecoration, "LX", box3DViewerid)
  box3DDecoration:setLineColor(0, 255, 0)
  box3DViewer:addShape(lineY, box3DDecoration, "LY", box3DViewerid)
  box3DDecoration:setLineColor(0, 0, 255)
  box3DViewer:addShape(lineZ, box3DDecoration, "LZ", box3DViewerid)

  resultViewer:present()
  connectedRegionViewer:present()
  connectedRegionEdgesViewer:present()
  box3DViewer:present()

  local endtime = DateTime.getTimestamp()
  Log.fine("processing time: " .. (endtime - startTime) .. " ms")
end

Script.register("Engine.OnStarted", main)
Image.Provider.Camera.register(camera, "OnNewImage", handleOnNewImage)
-- serve API in global scope
--End of Function and Event Scope-----------------------------------------------