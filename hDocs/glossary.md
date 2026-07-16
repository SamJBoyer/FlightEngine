<Terms>

<flight>

Nose-vector: the 3d unit vector that represents the direction the nose of the plane is pointing 

Translation-vector: the 3d unit vector that represents the translation the plane is experiencing. In flight, this often doesn't directly align with the nose-vector.

Alieron: wing-mounted control surfaces that control the roll of the plane

Rudder: tail mounted control surface that controls the yaw of the plane

Elevators: tail mounted control surfaces that control the pitch of the plane 

FCI (Flight controller input): a packet of alieron, rudder, and elevator instructions that represents the control-surface inputs of the plane.

VPC (Virtual Pilot Controller): Algorithm that translates the player's mouse input to FCI required to point the translation vector towards the Flight-cursor. 

Flight-cursor:  The in-flight cursor that indicates where the VPC is attempting to aim the translation-vector towards.

Input-bubble: the spherical surface surrounding the plane that is used to calculate the flight-cursor. Calculated as a projection from the camera through the plane onto the sphere. 

<flight-state>

Definition: a strucutre the represents the state of the plane. This object will be passed back and forth the boundry 

Contains:
- Position
- Rotation
- Linear velocity
- Angular velocity

</flight-state>
Flight-state: a struct that represents the aerodynamic state of the plane. Consists of position, rotation, linear velocity, and angular velocity. 

Flight-engine: Script responsible for taking a flight-state, applying an FCI to it, then calculating the resulting flight-state at each tick.

</flight>
<helmsman-terminology>

hDocs: shorthand for helmsman documents. 

itags: itag stands for issue-tags. issue-tags refer to the tag in the repository git issues page. Helmsman makes heavy use of git issues as a cheap and accessible way to create tickets related to the project. As a result, there are many different tags used to designate which issues mean which thing. For example, issues may be marked as ideas, features, bugs, etc. itag may have shared definitions between projects and repo-unique definitions. The itag section is the ground truth for what each tag means, which tags are used, and when to use each tag.

iticket: an instance of a git issue that has an itag

iPool: a document that contains a variably-structured list of thoughts, ideas, questions, and wants. Typically will live inside a lucidchart. Look at hDocs/remotes#lucidchart section to check for a remote iPool document.

critical desire: lives in the fPool. Is a 2 sentence summary of the most important desire the piece of infrastructure will fulfill. All wants in the 

critical question: a 2 sentence summary of the fundamental uncertainty that balances a critical desire

</helmsman-terminology>
<helmsman-version-control>

hVersion: The official version of helmsman currently used in this hProject OR the last official version used in this hProject. hVersion can be checked by looking at .helmsman/hVersion.md which contains the hTag 

hTag: the tag that indicates the hVersion. This matches an official canonical version found in the Helmsman github. The hTag will always be formatted vX.Y.Z and matches a git tag.

canonical version: an official helmsman release tagged on the github. These releases are finished, polished, and documented to offer a standardized starting point for new helmsman projects and a way to compare drift 
in an individual hProject to a baseline.

</helmsman-version-control>
<misc-features>

hlabel: stands for helmsman label. Uses colored emojis to mark certain ideas. [ ] 

hLabels are available and [X] hLabels are taken. If an hLabel is taken, it needs to have
a defined name and usage. 

</misc-features>
</Terms>

<iTags>
bug: something that is broken or behaving incorrectly. marked with the bug tag on git issues

quickssue: a quick, low-ceremony issue captured from casual input without deep investigation. marked with the quickssue tag on git issues

wonders: an open curiosity or "I wonder if..." question worth capturing for later exploration, not yet an idea or commitment. marked with the wonders tag on git issues
 
</iTags>
<hLabels>

[ ]: 🔴 #EA4335, [label], [usage]
[ ]: 🟠 #FF6D01, [label], [usage]
[ ]: 🟡 #FBBC04, [label], [usage]
[ ]: 🟢 #34A853, [label], [usage]
[ ]: 🔵 #4285F4, [label], [usage]
[ ]: 🟣 #9334E9, [label], [usage]
[ ]: 🟤 #A14236, [label], [usage]
[ ]: ⚫ #000000, [label], [usage]
[ ]: ⚪ #FFFFFF, [label], [usage]
[ ]: 🟥 #EA4335, [label], [usage]
[ ]: 🟧 #FF6D01, [label], [usage]
[ ]: 🟨 #FBBC04, [label], [usage]
[ ]: 🟩 #34A853, [label], [usage]
[ ]: 🟦 #4285F4, [label], [usage]
[ ]: 🟪 #9334E9, [label], [usage]
[ ]: 🟫 #A14236, [label], [usage]
[ ]: ⬛ #000000, [label], [usage]
[ ]: ⬜ #FFFFFF, [label], [usage]

</hLabels>