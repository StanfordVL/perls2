# Replay Example

## Description
The example builds off the simple reach example and shows how to record demonstrations and then playback actions to a restored environment state. Action Playback is useful for imitation learning and behavior cloning applications. This check can be used to verify determinism using the PyBullet simulator. 

## Run file:
The Simple Reach Environment is extended to return flattened observations of the robot proprio state and the object state. These observations will be used to compare recorded demo states and resulting playback states. If the replay state is 100% deterministic, every observation between the two should be **exactly** equal. 

The example also shows the option to save these states to a .bullet file or in-memory. 
