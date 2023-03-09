Built using JMonkeyEngine.

# How to run

In the project root, run `gradlew build` to build the jar file. The output file bundles all required dependencies automatically.

Then, run `cd build/libs` and then `start.bat`. `start.bat` uses the Java on your PATH, and sets a few helpful JVM arguments for increased speed (along with setting 6gb of heap memory so it doesn't crash on start).

Note that this takes 4 minutes each time to build, so make wise changes.

Also included in this codebase is MathUtil, ForwardKinematicsUtil, and InverseKinematicsUtil copied verbatim from the robot codebase (though I have added one or two methods to MathUtil). Constants has important sections copied verbatim.

Any working updates to the Utility classes or to Constants should be ported to the robot codebase.