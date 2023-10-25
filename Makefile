default: docs

docs:
	cd rai-docs && sphinx-build doc ../html

docs-clean:
	rm -Rf html

install:
	ln -f -s _build_utils/CMakeLists-ubuntu.txt CMakeLists.txt
	cmake -B build .
	+make -C build _robotic docstrings install

install-clean:
	rm -R ${HOME}/.local/lib/python3.8/site-packages/robotic*
	rm ${HOME}/.local/lib/*rai*
	rm ${HOME}/.local/bin/*ry*

wheels:
	$(eval id = $(shell _build_utils/run-docker.sh -d))
	@echo "started docker " ${id}
	docker exec -it ${id} /bin/bash -C local/_build_utils/build-wheels.sh
	docker stop ${id}
#	_build_utils/run-docker.sh local/_build_utils/build-wheels.sh

wheels-push:
	twine upload dist/*.whl --repository robotic

wheels-local:
	python3.8 -m pip install --user dist/robotic-*cp38*.whl --force-reinstall

test:
	cd ${HOME} && python3 -c 'from robotic import ry; print("ry version:", ry.__version__, ry.compiled());'

pull:
	cd rai && git pull
	cd rai-robotModels && git pull
	cd rai-docs && git pull
	cd botop && git pull

docker-clean:
	$(shell docker container kill "$(docker container ls -q)")
	docker system prune

