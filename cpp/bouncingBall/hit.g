frame world{}

frame wall1{ shape=9, X=<T t(.0 0 .5)>, size=[2. 2. .1 .04], color=[.3 .3 .3 1.] fixed, contact }


frame ball (wall1) {
    size=[.1 .1 .1 .1] color=[.2 .2 .8 1.] shape=sphere contact
    from=<T t(0 0 .15)> joint=free, logical={ object },
    mass = 1, dynamic }

frame ball2 (wall1) {
    size=[.1 .1 .1 .1] color=[.8 .2 .2 1.] shape=sphere contact
    from=<T t(.3 .6 .15)> joint=free, logical={ object },
    mass = 1, dynamic }

frame target (wall1) {
    shape=ssBox size=[.3 .3 .04 .01] color=[.3 .6 0],
    joint=rigid
    from=<T t(-.5 -.5 .03)>, logical={ table }
}
