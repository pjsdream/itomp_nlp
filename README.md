# itomp_nlp
NLP + motion planning

## Current Status
Doing OpenGL tutorials for nice-looking rendering

## Dependencies
* Qt5 (https://www.qt.io/developers/, tested on Qt5.7)
* lodepng (http://lodev.org/lodepng/)
* tinyxml2 (https://github.com/leethomason/tinyxml2)
* assimp (http://www.assimp.org/, http://www.assimp.org/lib_html/index.html)
* Fetch robot urdf

Copy the following lib files to lib/ folder
* lodepng.cpp
* lodepng.h
* tinyxml2.cpp
* tinyxml2.h
* assimp-vc140-mt.lib (on Windows)

Copy the following dll files to bin/ folder (on Windows)
* Qt5Core.dll
* Qt5Gui.dll
* Qt5Widgets.dll
* assimp-vc140-mt.dll

Copy urdf files to urdf/ folder
Copy mesh files to meshes/ folder