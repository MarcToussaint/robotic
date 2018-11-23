frame world{}

frame boxBo{ shape=9, X=<T t(.0 0 .5)>, size=[2. 2. .1 .04], color=[.3 .3 .3 .8] fixed, contact }

frame boxLe{ shape=9, X=<T t(-1. 0 1.5) d(90 0 1 0)>, size=[2. 2. .1 .04], color=[.3 .3 .3 .8] fixed, contact }

frame boxBa{ shape=9, X=<T t(0 1. 1.5) d(90 1 0 0)>, size=[2. 2. .1 .04], color=[.3 .3 .3 .8] fixed, contact }


### ball

#frame block{ shape=ssBox, X=<T t(.1 0 .8)>, size=[.5 .2 .3 .02], color=[.9 .3 .3 .7] fixed, contact }
#joint (world block){ type=free Q=<T t(-1.1 0 1.) d(10 1 1 0)> }


#frame block2{ shape=ssBox, X=<T t(.1 0 .8)>, size=[.5 .2 .3 .02], color=[.9 .3 .3] fixed, contact }
#joint (world block2){ type=free Q=<T t(-.1 -.5 1.1)> }

frame marker1{ shape=marker color=[1 0 0] size=[.2 0 0] }
frame marker2{ shape=marker color=[1 1 0] size=[.2 0 0] }


frame ball (world) {
    size=[.1 .1 .1 .1] color=[.2 .2 .8 .8] shape=sphere contact
    from=<T t(0 0 1.)> joint=free, logical={ object },
    mass = 1, dynamic }

frame target (boxBa) {
    shape=ssBox size=[.3 .3 .04 .01] color=[.3 .6 0],
    joint=rigid
    from=<T t(0 0 .05)>, logical={ table }
}
