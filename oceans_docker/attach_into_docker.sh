#!/bin/bash

sudo docker exec -it $(sudo docker ps -aql) bash
