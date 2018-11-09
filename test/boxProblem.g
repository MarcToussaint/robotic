frame base{}

### box
box { X=<t(.0 0 .5)> }

boxBo (box){ shape=ssBox, Q=<t(.0 .1 0)>, size=[1.5 2.8 .3 .04], color=[.2 .2 .2 .3]  contact=1 }

#boxTo (box){ shape=ssBox, Q=<t(.0 0 1.5)>, size=[1.5 2. .1 .04], color=[.2 .2 .2 .3]  contact }

boxLe (box){ shape=ssBox, Q=<t(-.75 0 .75) d(90 0 1 0)>, size=[1.5 2. .1 .04], color=[.2 .2 .2 .3] contact }

#boxRe (box){ shape=ssBox, Q=<t( .75 0 .75) d(90 0 1 0)>, size=[1.5 2. .1 .04], color=[.2 .2 .2 .3] contact }

boxBa (box){ shape=ssBox, Q=<t(0 1. .75) d(90 1 0 0)>, size=[1.5 1.5 .1 .04], color=[.2 .2 .2 .3] contact }

#start (box){ shape=ssBox, Q=<t(.0 -1. .05)>, size=[1.5 .4 .1 .04], color=[.2 .2 .2 .5]  contact }

#rail (start){ shape=ssBox, Q=<t(.0 -.15 .05)>, size=[1.5 .1 .1 .04], color=[.2 .2 .2 .5]  contact }


### objects

#ballB (box) {
#    size=[0 0 0 .05] color=[.2 .2 .8 1] shape=sphere contact
#    Q=<t(0 0 .1)> logical={ object },
#    mass = 1 }

ballR (boxBo) {
    size=[0 0 0 .05] color=[.8 .2 .2 1] shape=sphere contact
    Q=<t(.3 -1. 1.)> logical={ object },
    joint=rigid,
    mass = 1, dynamic }

block (box) {
    size=[.2 .1 .1 .01] color=[.2 .2 .8 1] shape=ssBox contact
    Q=<t(-.3 -1. 1.) d(20 0 1 0)> logical={ object },
    joint=rigid,
    mass = 1, dynamic }

### hook

#stick (start) {
#    Q=<t(0 -.15 .07)>
#    shape=ssBox size=[.8 .025 .04 .01] color=[.6 .3 0] contact, logical={ object } }


target (boxBa) {
    shape=ssBox size=[.3 .3 .04 .01] color=[.3 .6 0],
    joint=rigid
    from=<t(0 0 .05)>, logical={ table }
}
