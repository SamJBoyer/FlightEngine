<summary>

This project is the site of my custom game engine for flight control for a pseudo-realistic War Thunder flying game. 

Because I'm not sure which degree of sophisitcation/realism is needed to create all the behaviours I want, so I am going to have multiple engines developed, tuned, and tested and I'll see which one feels the best. Each flight engine is supposed to use the plane controller system. THe user will list the physics features they want and a system should be tailored to meet them. 

It is structured as a clean C# project to cut down on token bloat.

This game engine is going to have a front end/back end structure where this becomes part of a stand-alone project that powers a novel front end. Because of this, we need to pay special attention to what information needs to cross the boundary layer.

hDocs/boundry.md explains the logic in greater detail

Grok took a shot at this and did an amazing job. The physics look good and there is even a decent visualizer. 

</summary> 
<remotes>

FlightEngine-iPool stored at My Documents/OrkEngines/FlightEngine/FlightEngine-iPool


</remotes>
<artifacts>

All features are currently looking pretty good. 

</artifacts>
<drift>

As far as I can tell Grok has hit every need. The test suite isn't amazing so there could be some bubbles. 

</drift>
<overlay>

This project was created with Helmsman. Read .helmsman/hVersion.md to see the canonical version tag. 

Project is also half of the PigFlight project to create a War Thunder arcade-like pseudo-realistic physics engine for my game. This repo only creates the physics engine, which is then passed to a seperate frontend in a different project. 

</overlay>
<assumptions>

Initialized as empty 

</assumptions>

