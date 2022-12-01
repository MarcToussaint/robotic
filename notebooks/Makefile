
ipynb_paths =  $(shell find . -type f -name '*.ipynb' -not -name '*checkpoint*' -printf "%f ")

run_paths = 1-basics.ipynb 2-cameraView.ipynb 4-path-optimization.ipynb 2-KOMO-switches.ipynb ry-skeletons.ipynb

run:
	@for p in $(run_paths); do jupyter-nbconvert --to notebook --execute $$p; done

clean: $(ipynb_paths:%=clean/%)

clean/%.ipynb: %.ipynb
	+@-jupyter-nbconvert --clear-output --inplace $<

run/%.ipynb: %.ipynb
	+@-jupyter-nbconvert --to notebook --execute $<
