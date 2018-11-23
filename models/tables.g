#Edit worldTranslationRotation { ctrl_H=1 }

Frame table1{ shape:ssBox, X=<T t(0 2. .6) d(90 0 0 1)>, size=[.8 2.2 .1 .02], color=[.3 .3 .3] fixed, contact, logical={  } }

Frame table2{ shape:ssBox, X=<T t(2. 0 .6)>, size=[.8 2.2 .1 .02], color=[.3 .3 .3] fixed, contact, logical={ table } }

Frame table3{ shape:ssBox, X=<T t(0 -2. .6) d(90 0 0 1)>, size=[.8 2.2 .1 .02], color=[.3 .3 .3] fixed, contact, logical={     } }
