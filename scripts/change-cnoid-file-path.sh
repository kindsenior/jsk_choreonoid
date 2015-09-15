#!/bin/bash

function change-cnoid-file-path-recursive(){
		echo $val
		for val in $1/*
		do
				if [ -d $val ]
				then
						echo $val " is directory"
						change-cnoid-file-path-recursive $val
				elif [ ${val##*.} = "cnoid" ]
				then
						echo $val " is cnoid file"
						sed $val -e s/hydro/indigo/g | cat > ${val/./_.}
						mv ${val} /tmp/
						mv ${val/./_.} ${val}
				fi
		done
}

change-cnoid-file-path-recursive ${HOME}/Dropbox/choreonoid/
