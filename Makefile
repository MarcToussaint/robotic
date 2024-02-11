PY_VER = $(shell python3 -c "import sys; print(str(sys.version_info[0])+'.'+str(sys.version_info[1]))")

default: docs

docs:
	cd rai-docs && sphinx-build doc ../html

docs-clean:
	rm -Rf html

local-install:
	ln -f -s _build_utils/CMakeLists-ubuntu.txt CMakeLists.txt
	-rm -f ${HOME}/.local/lib/*rai*
	cmake . -B build
	+make -C build _robotic docstrings install
	cp build/_robotic.pyi ${HOME}/.local/lib/python$(PY_VER)/site-packages/robotic

local-clean:
	-rm -Rf ${HOME}/.local/lib/python$(PY_VER)/site-packages/robotic*
	-rm -f ${HOME}/.local/lib/*rai*
	-rm -f ${HOME}/.local/bin/*ry*

wheels:
	$(eval id = $(shell _build_utils/run-docker.sh -d))
	@echo "started docker " ${id}
	docker exec -it ${id} /bin/bash -C local/_build_utils/build-wheels.sh
	docker stop ${id}
#	_build_utils/run-docker.sh local/_build_utils/build-wheels.sh

wheels-upload:
	twine upload dist/*.whl --repository robotic

wheels-install:
	python$(PY_VER) -m pip install --user dist/robotic-*cp38*.whl --force-reinstall

test:
	cd ${HOME} && python3 -c 'import robotic as ry; print("ry version:", ry.__version__, ry.compiled());'

test-tutorials:
	make -j1 -C rai-docs/rai-tutorials run

pull:
	cd rai && git pull
	cd rai-robotModels && git pull
	cd rai-docs && git pull
	cd rai-docs/rai-tutorials && git pull
	cd botop && git pull

docker-clean:
	$(shell docker container kill "$(docker container ls -q)")
	docker system prune

