Include = '../rai-robotModels/baxter/baxter.g'

Edit base { X=<T t(0 0 .7)> } # for kuka
Edit base_footprint { X=<T t(-.4 0 0)> } # for baxter

body slider1a { type=box size=[.2 .02 .02 0] color=[.5 .5 .5 .0] }
body slider1b { type=box size=[.2 .02 .02 0] color=[.8 .3 .3 .0] }
joint slider1Joint(slider1a slider1b){ type=transX ctrl_H=.1 }
shape (slider1b){ rel=<T t(.1 0 0)> type=5 size=[.1 .1 .1] color=[0 1 0] }

body table1{ type=9, X=<T t(.8 0 .7)>, size=[2. 3. .04 .02], color=[.3 .3 .3] fixed, contact, logical={ table } }

#Edit baxterL{ logical={ gripper, free } }
#Edit baxterR{ logical={ gripper, free } }

Edit baxterL{  shape=ssBox size=[.03 .03 .06 .01] rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26) t(0 0 .05)> logical={ gripper, object, moves } }
Edit baxterR{ shape=ssBox size=[.03 .03 .06 .01] rel=<T d(-90 0 1 0) d(-90 0 0 1) t(0 0 -.26) t(0 0 .05)> logical={ gripper, object, moves } }

Edit head_pan { type=10 }
Edit r_gripper_l_finger_joint { type=10 }
Edit r_gripper_r_finger_joint { type=10 }
Edit l_gripper_l_finger_joint { type=10 }
Edit l_gripper_r_finger_joint { type=10 }

### ball

body redBall { size=[.06 .06 .06 .02] color=[1 0 0] type=ssBox contact, logical={ object } }
joint (table1 redBall) { from=<T t(0 0 .02) t(.0 .7 .03)> type=rigid }

### hook

body nostick { type=5 size=[.2 .2 .2] }
joint (table1 nostick) { from=<T t(0 0 .02) t(-.2 -.7 .02)> type=rigid }
shape stick(nostick) { type=ssBox size=[.8 .025 .04 .01] color=[.6 .3 0] contact, logical={ object } }
shape stickTip (nostick) { rel=<T t(.4 .1 0) d(90 0 0 1)> type=ssBox size=[.2 .026 .04 0.01] color=[.6 .3 0], logical={ object, pusher } }
