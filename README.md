# FreeFormMotionProcessing
Implementation of the interpolation section of ["Free Form Motion Processing"](http://mgarland.org/files/papers/freeform.pdf) - Kircher &amp; Garland - 2008

Use case example:

We have two 3D frames of the same horse in motion (look at the legs):

<img src="https://raw.githubusercontent.com/paghdv/FreeFormMotionProcessing/master/samples/horse1.png" alt="horse1" style="width: 300px;"/><img src="https://raw.githubusercontent.com/paghdv/FreeFormMotionProcessing/master/samples/horse2.png" alt="horse2" style="width: 250px;"/>

<br/>
If we simple interpolate these two frames using cartesian coordinates we obtain:

<img src="https://raw.githubusercontent.com/paghdv/FreeFormMotionProcessing/master/samples/cartesian.png" alt="cartesian" style="width: 500px;"/>
<br/>

There is a clear problem of a loos of volume in the legs, plus the motion itself being lost in the interpolation. Instead, if we use the mesh representation and interpolation proposed in "Free Form Motion Processing" we obtain:

<img src="https://raw.githubusercontent.com/paghdv/FreeFormMotionProcessing/master/samples/ffmp.png" alt="cartesian" style="width: 500px;"/>

