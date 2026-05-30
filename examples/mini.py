import robotic as ry
import time

C = ry.Config()
C.addFile('$RAI_PATH/panda/panda.g')


# RealSense/captureDepth: false
# RealSense/startupSkipImages: 1
# RealSense/resolution: 640
# RealSense/alignToDepth: false
# RealSense/autoExposure: true
# RealSense/exposure: 100
# RealSense/white: 3000
# #RealSense/serial_number_0: "102422071099"
# #RealSense/serial_number_1: "825312070938"
# RealSense/serial_number_0: "341222301882"
# RealSense/serial_number_1: "338522301836"

ry.set_params({
    'bot/forceRealCamera': True,
    'RealSense/captureDepth': False,
    'RealSense/startupSkipImages': 1,
    'RealSense/resolution': 640,
    'RealSense/alignToDepth': False,
    'RealSense/autoExposure': True,
    'RealSense/exposure': 100,
    'RealSense/white': 3000,
    'RealSense/serial_number_0': "341222301882",
    'RealSense/serial_number_1': "338522301836"
})

bot = ry.BotOp(C, False)

view = C.viewer()
for i in range(100):
    img0, _ = bot.getImageAndDepth('RealSense_0')
    img1, _ = bot.getImageAndDepth('RealSense_1')
    view.setQuad(0, img0, 0.1, 0.1, 0.4)
    view.setQuad(1, img1, 0.6, 0.1, 0.4)
    C.view(False)
    print(i)
    time.sleep(.01)
