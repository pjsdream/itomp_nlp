# itomp_nlp
NLP + motion planning

## Current Status
Robot rendering with OpenGL
Cosine similarity measurement between words
Gradient method trajectory optimization
Cost functions
ROS catkin setup

## Dependencies
* Qt5 (https://www.qt.io/developers/, tested on Qt5.7)
* lodepng (http://lodev.org/lodepng/)
* tinyxml2 (https://github.com/leethomason/tinyxml2)
* assimp 3.3.1 (http://www.assimp.org/, http://www.assimp.org/lib_html/index.html)
* Fetch-ros (https://github.com/fetchrobotics/fetch_ros)
* GloVe pre-trained word-to-vector data (http://nlp.stanford.edu/projects/glove/, glove.6B.zip)

Copy the following lib files to lib/ folder
* lodepng.cpp
* lodepng.h
* tinyxml2.cpp
* tinyxml2.h

Copy urdf files to urdf/ folder

### Windows
Copy the following dll files to bin/ folder
* Qt5Core.dll
* Qt5Gui.dll
* Qt5Widgets.dll
* assimp-vc140-mt.lib
* assimp-vc140-mt.dll

### Linux
Nothing to do.