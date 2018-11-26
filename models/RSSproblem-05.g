Include = 'RSSproblem-shared.g'

#Edit table1{ X=<T t(1. 0 .7)>, size=[2.4 2. .04 .02] }

### ball

frame blueBall (table1) { size=[.1 .1 .1 .05] color=[.2 .2 .8] shape=ssBox contact
 from=<T t(0 0 .02) t(-.3 .5 .05)> joint=rigid , logical={ object } }

### tools

body stick1 {}
joint (table1 stick1) { from=<T t(0 0 .02) t(-.5 -.5 .03)> type=rigid}
shape stick(stick1) { type=ssBox size=[.8 .07 .07 .03] color=[.6 .3 0] contact, logical={ object } }


### bucket

frame bucket (table1) {
    shape=ssBox size=[.3 .3 .04 .01] color=[.3 .6 0] contact,
    joint=rigid from=<T t(0 0 .02) t(.5 -.8 -.0195)>
    , logical={ table }
}
