frame world{}

frame table1{ shape=9, X=<T t(.0 0 .73) d(30 0 1 0)>, size=[2. 2. .5 .052], color=[.3 .3 .3 .7] fixed, contact }


### ball

frame block{ shape=ssBox, size=[.5 .2 .3 .1], color=[.9 .3 .3 .7] contact, mass=1, dynamic }
joint (world block){ type=free Q=<T t(0 0 2.)> }


#frame block2{ shape=ssBox, X=<T t(.1 0 .8)>, size=[.5 .2 .3 .02], color=[.9 .3 .3] fixed, contact }
#joint (world block2){ type=free Q=<T t(-.1 -.5 1.1)> }

#frame marker1{ shape=marker color=[1 0 0] size=[.2 0 0] }
#frame marker2{ shape=marker color=[1 1 0] size=[.2 0 0] }

