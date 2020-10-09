# Examples: Objects

## Description
This example shows the different ways to work with objects within a perls2 environment. Object interfaces are specific to simulation environments as it's nontrivial to obtain object state in the real world. There are two ways to load objects in perls2: either automatically from a configuration file or within an environment.

## Loading objects from config file:
To load objects, you can specify them in the config under the 'objects' as an object_dict. If you don't want to specify any objects using the config file,
just comment out the object_dict key and everything undernearth.

### Data directories
By default, the data_dir key is commented out. If you want to specify your own urdfs, meshes etc.for objects you can specify this directory with an absolute filepath. This will append the value of the data_dir key to the object filepaths
set in the config.

## Loading object files with add_objects.
You can also load the objects in python using the `perls2.world.add_object()` function. This is helpful for scripting or when you don't want to specify each object in the config.

## Remove objects with remove_objects:
You can remove an object from the simulation using its identifying key in the world.object_interfaces using remove_objects.




