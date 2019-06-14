## Building thje image
 ```
 docker build -t 3duse 18.04
 ```

## Running the client on OSX
References:
 * https://stackoverflow.com/questions/37826094/xt-error-cant-open-display-if-using-default-display
 * https://stackoverflow.com/questions/16296753/can-you-run-gui-applications-in-a-docker-container

Start by installing XQuartz and then
```
/Applications/Utilities/XQuartz.app/Contents/MacOS/X11.bin &
brew install socat
socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\" &
docker run --rm -e DISPLAY=docker.for.mac.host.internal:0 3duse
```
alas as for version 2.7.11 of XQuartz, and as commented on this
[ask-ubuntu thread](https://askubuntu.com/questions/771000/ssh-from-my-mac-into-ubuntu-x-forwarding-not-working-correctly) XQuartz has a problem and we get the following run time error message
```
X Error: BadValue (integer parameter out of range for operation) 2
  Extension:    149 (Uknown extension)
  Minor opcode: 3 (Unknown request)
```
It seems the XQuartz team is understaffed and this problem won't go away any time
soon. As suggested on the above mentioned thread the solution might be to
use a VNC-viewer as opposed to X11 technology.
