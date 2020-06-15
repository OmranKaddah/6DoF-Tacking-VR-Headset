<h1>6DoF TRACKING OF A VR HEADSET</h2>
<h3>Discrption</h3>
<p>Project consists of a a system which tracks a model with ArUco markers attached on it, which designed to be mounted on the VR headset for the purpose of this. The tracking system estemates the 3D pose of the calibrated camera with respect to the n points on the model by solving PnP(Perspective-n-Points) problem after the detection of ArUco markers. The resulting information of the tracking system, the rotation (roll, pitch, and yaw) and 3D translation of the camera with respect to the world, is sent to the mobile device(Andriod). The information can be used in any 6DoF VR application for detecting user's headmovments and orientation.</P


<h3> Crafting The Mounted Model</h3>
<p>
The 3D mounted model can found in the file Model.skp . The model can be viewed using SketchUp (can be downloaded for free). All the required measurements can be done that model.
</p>

<p> 
It is better to craft the model using low-wight metrials like hard cardbord. And the VR headset should fit in the model. Otherwise the dimension constants have to be changed in the code.
</p>

<p> Markers to attached are found in the folder Markers. The picture of each marker is in the excat right dimension of the code. Each edge should have a lenght of 6 cm. And every markers has a code, the corrosponding code is the name of the file. Each marker should be placed according the to blueprint of the model. The model blueprint can found in ModelP.jpg
</p>

<p> After building the project and debugging it and after crafting the model. If the model is placed in fornt of camera, the resultant information will be send wirelessly, there is an app in Andriod folder, which displays the translationa nd rotation vectors given the IP address and the port(54001). The information is sent in the pattern  X;Y;Z;roll;pitch;yaw;</p>

<p> The project Requires importing OpenCv Library and contribution library of Opencv, when building the libraries, with Cmake for instance, include ArUco.</p>

<p> For more info about project please check the report</p>
