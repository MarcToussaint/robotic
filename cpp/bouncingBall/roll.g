frame world{}

frame wall1{ shape=9, X=<T t(.0 0 .5)>, size=[2. 2. .1 .04], color=[.3 .3 .3 .8] fixed, contact }

frame wall2{ shape=9, X=<T t(.3 .6 1.3) d(30 1 0 0)>, size=[1.6 1.6 .1 .04], color=[.3 .3 .3 .8] fixed, contact }

frame ball (world) {
    size=[.1 .1 .1 .1] color=[.2 .2 .8 .8] shape=sphere contact
    from=<T t(0 0 1.)> joint=free, logical={ object },
    mass = 1, dynamic }

frame target (wall1) {
    shape=ssBox size=[.3 .3 .04 .01] color=[.3 .6 0],
    joint=rigid
    from=<T t(-.5 -.5 .05)>, logical={ table }
}
