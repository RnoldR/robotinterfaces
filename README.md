# Basic commands

## 1. Create new package

What makes a ROS package?


-    package.xml file containing meta information about the package
-    resource/<package_name> marker file for the package
-   setup.cfg is required when a package has executables, so ros2 run can find them
-   setup.py containing instructions for how to install the package
-    <package_name> - a directory with the same name as your package, used by ROS 2 tools to find your package, contains __init__.py

The simplest ROS package contains the following files:

```
my_package/
      package.xml
      resource/my_package
      setup.cfg
      setup.py
      my_package/
```

Best practice is to create new packages in the workspace/src directory. Mirrors will be setuop for build, log and install directories.

```
workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
      ...
      cpp_package_n/
          CMakeLists.txt
          include/cpp_package_n/
          package.xml
          src/
```

### Create a package

```
cd workspace_folder/src
ros2 pkg create --build-type ament_python <package_name>
```

Use the optional argument `--node-name` to creates a 
simple Hello World type executable in the package.

```
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

## 2. Build a package

```
colcon build --packages-select my_package
```

## 3. Source the setup file

Add the workspace to the path.

```
source install/local_setup.bash
```

## 4. Run the package

To run the executable you created using the --node-name argument during package creation, enter the command:

```
ros2 run my_package my_node
```

## 5. Examine package contents

nside ros2_ws/src/my_package, you will see the files and folders that ros2 pkg create automatically generated: `my_package  package.xml  resource  setup.cfg  setup.py  test`.

## 6. Customize the package

### package.xml

```
<?xml version="1.0"?>
<?xml-model
   href="http://download.ros.org/schema/package_format3.xsd"
   schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
 <name>my_package</name>
 <version>0.0.0</version>
 <description>TODO: Package description</description>
 <maintainer email="user@todo.todo">user</maintainer>
 <license>TODO: License declaration</license>

 <test_depend>ament_copyright</test_depend>
 <test_depend>ament_flake8</test_depend>
 <test_depend>ament_pep257</test_depend>
 <test_depend>python3-pytest</test_depend>

 <export>
   <build_type>ament_python</build_type>
 </export>
</package>
```

Change at least the following lines:

- `maintainer` - maintainer email and name
- `description` - give a good description of the package
- `license` - apply the coorect license
- `_depend` - dependencies of the package

### setup.py

Change 

- `maintainer` - maintainer name
- `maintainer_email` - maintainer email
- `description` - give a good description of the package
- `license` - apply the coorect license
- `entry_points` - supply the `console_scripts` with which to start yout package

```
    entry_points={
        'console_scripts': [
                'my_node = my_py_pkg.my_node:main'
        ],
    },
```

