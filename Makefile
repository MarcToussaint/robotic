PY_VER = $(shell python3 -c "import sys; print(str(sys.version_info[0])+'.'+str(sys.version_info[1]))")
PY_SITE = $(shell python3 -m site --user-site)

default: compile

compile:
	make -C build

docs:
	cd rai-docs && sphinx-build doc ../html

docs-clean:
	rm -Rf html

local-install:
	ln -f -s _build_utils/CMakeLists-ubuntu.txt CMakeLists.txt
	-rm -f ${HOME}/.local/lib/*rai*
	cmake . -B build -DPY_VERSION=$(PY_VER)
	+make -C build _robotic docstrings install -j $(command nproc --ignore 2)
	cp build/_robotic.pyi $(PY_SITE)/robotic

local-clean:
	-rm -Rf $(PY_SITE)/robotic
	-rm -Rf $(PY_SITE)/robotic-*
	-rm -f ${HOME}/.local/lib/*rai*
	-rm -f ${HOME}/.local/bin/*ry*

wheels:
	$(eval id = $(shell _build_utils/run-docker.sh -d))
	@echo "started docker " ${id}
	docker exec -it ${id} /bin/bash -C local/_build_utils/build-wheels.sh
	docker stop ${id}
#	_build_utils/run-docker.sh local/_build_utils/build-wheels.sh

wheels-upload:
	twine upload dist/*.whl

wheels-install:
	python$(PY_VER) -m pip install --user dist/robotic-*cp310*.whl --force-reinstall

test:
	cd ${HOME} && python3 -c 'import robotic as ry; print("ry version:", ry.__version__, ry.compiled());'

test2:
	cd ${HOME} && python3 -c 'import robotic as ry; ry.test.RndScene()'

test3:
	ry-view $(PY_SITE)/robotic/rai-robotModels/scenarios/pandasTable.g

test-tutorials:
	make -j1 -C rai-tutorials run

pull:
	cd rai && git pull
	cd rai-robotModels && git pull
	cd rai-docs && git pull
	cd rai-tutorials && git pull
	cd botop && git pull

docker-clean:
	$(shell docker container kill "$(docker container ls -q)")
	docker system prune

