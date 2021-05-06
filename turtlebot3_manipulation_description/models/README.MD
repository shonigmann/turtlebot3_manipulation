This folder will generate symlinks to xacro files, meshes, etc in the Turtlebot3 and OpenManipulatorX Description folders. This will help reduce the number of duplicate robot descriptions while satisfying Gazebo's model path requirements

Run:
python generate_symbolic_model_library.py 

To populate this folder. 

TODO: have this script run every time this package is built, to ensure the models are up to date
