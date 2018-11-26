Include = 'RSSproblem-shared.g'

### ball

body redBall { size=[.06 .06 .06 .02] color=[1 0 0] type=ssBox contact, logical={ object } }
joint (table1 redBall) { from=<T t(0 0 .02) t(.0 .7 .03)> type=rigid }

### hook

body nostick { type=5 size=[.2 .2 .2] }
joint (table1 nostick) { from=<T t(0 0 .02) t(-.2 -.7 .02)> type=rigid }
shape stick(nostick) { type=ssBox size=[.8 .025 .04 .01] color=[.6 .3 0] contact, logical={ object } }
shape stickTip (nostick) { rel=<T t(.4 .1 0) d(90 0 0 1)> type=ssBox size=[.2 .026 .04 0.01] color=[.6 .3 0], logical={ object, pusher } }
