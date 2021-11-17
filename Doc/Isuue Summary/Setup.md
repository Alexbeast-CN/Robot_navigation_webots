# Webots setup for Linux

## 1. Download 

> ref:[Download Webots on linux](https://cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)

Use snap to download Webots. 

By using `gedit ~/.bashrc` to open the text. Then add `export WEBOTS_HOME=/snap/webots/current/usr/share/webots:$WEBOTS_HOME # Defines the path to Webots home.` to the end of the text.

In Vscode, add `"${WEBOTS_HOME}/include/controller/cpp/**"` to `"includePath"` in the file called `c_cpp_properties.json`.

## 2. Connect Github with Linux

1. Generate a SSH code on your Linux end.
2. Add your Linux PC to your Gitub account.
3. Setup vscode on Linux.
4. Git clone with ssh.

# Set a C++ frame to main function

