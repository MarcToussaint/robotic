
ipynb_paths =  $(shell find . -type f -name '*.ipynb' -not -name '*checkpoint*' -printf "%f ")

#run:
#	@for p in $(ipynb_paths); do jupyter-nbconvert --to notebook --execute $$p; done

run: $(ipynb_paths:%=run/%)
clean: $(ipynb_paths:%=clean/%)
convert: $(ipynb_paths:%=convert/%)

clean/%.ipynb: %.ipynb
	+@-jupyter-nbconvert --clear-output --inplace $<

convert/%.ipynb: %.ipynb
	+@-jupyter-nbconvert --to python $<

run/%.ipynb: %.ipynb
	+@-echo "=========== run $< ======="
	+@-jupyter-nbconvert --to notebook --execute $<
