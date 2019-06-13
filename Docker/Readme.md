Usage example:
 '''
 docker build -t 3duse 18.04
 '''

Running on OSX
 - References:
    * https://stackoverflow.com/questions/37826094/xt-error-cant-open-display-if-using-default-display
    * https://stackoverflow.com/questions/16296753/can-you-run-gui-applications-in-a-docker-container
'''
/Applications/Utilities/XQuartz.app/Contents/MacOS/X11.bin &
brew install socat
socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\" &
docker run --rm -e DISPLAY=docker.for.mac.host.internal:0 3duse
'''
