#!/bin/bash

if [ "$1" == "-h" ] || [ "$1" == "--h" ] || [ "$1" == "-help" ] || [ "$1" == "--help" ]; then
    echo "Usage -> bash absolute/path/to/run_experiments.sh path/to/datasets/folder path/to/binaries"
    exit
fi

SCRIPT_NAME=`basename $0`
SCRIPT_DIR=`dirname $0`
CONF_DIR=${SCRIPT_DIR}/conf
TARGET_DIR=$1
BIN_DIR=$2
CURRENT_DIR=${PWD}

if [ "$3" == "clean" ]; then
    echo -n "Removing previous results..."
    # rm -rf logs/gicp*.txt logs/dvo*.txt logs/kinfu*.txt logs/nicp*.txt logs/ndt*.txt
    # rm -rf results/gicp*.txt results/dvo*.txt results/kinfu*.txt results/nicp*.txt results/ndt*.txt
    # rm -rf results/gicp*.png results/dvo*.png results/kinfu*.png results/nicp*.png results/ndt*.txt
    # rm -rf plots/gicp*.txt plots/dvo*.txt plots/kinfu*.txt plots/nicp*.txt plots/ndt*.txt
    
    # mkdir logs
    # mkdir results
    # mkdir plots
    echo " done"
fi
rm -rf results/computation_times.txt results/mean_computation_times.txt
rm -rf results/rpe* results/ate*

LOGS_DIR=${CURRENT_DIR}/logs
RESULTS_DIR=${CURRENT_DIR}/results
PLOTS_DIR=${CURRENT_DIR}/plots

echo "Running script: ${SCRIPT_NAME}"
echo "Script directory: ${SCRIPT_DIR}"
echo "Conf directory: ${CONF_DIR}"
echo "Target directory: ${TARGET_DIR}"
echo "Binaries directory: ${BIN_DIR}"
echo "Current directory: ${CURRENT_DIR}"
echo "Logs directory: ${LOGS_DIR}"
echo "Results directory: ${RESULTS_DIR}"

xterm -e roscore &

for f in ${TARGET_DIR}/*;
do 
    if [[ -d "${f}" && ! -L "${f}" ]]; then
	DATASET_NAME=`basename ${f}`
	if [ "${DATASET_NAME}" == "logs" ] || [ "${DATASET_NAME}" == "results" ] || [ "${DATASET_NAME}" == "plots" ]; then
	    continue
	fi	
	echo "----------------------------------------------------------"
	cd ${f}
	echo "Evaluating ${DATASET_NAME}"
	echo "Entering ${f}"
	##################################################################### 
	##                          Run Benchmarks                         ##
	#####################################################################
	if [ "$3" == "clean" ]; then
	    echo -n "Generating associations..."
	    python ${SCRIPT_DIR}/associate.py depth.txt rgb.txt > associations.txt		
	    echo " done"
	    
	    echo -n "Running GICP..."
	    ${BIN_DIR}/nicp_eth_kinect_evaluate ${CONF_DIR}/gicp_eth_kinect.conf associations.txt ${RESULTS_DIR}/gicp_odometry_${DATASET_NAME}.txt 0 > ${LOGS_DIR}/gicp_log_${DATASET_NAME}.txt
	    echo " done"
	    # echo -n "Running DVO..."
	    # roslaunch dvo_benchmark benchmark.launch output_dir:=${RESULTS_DIR} dataset:=${PWD} associations_file:=associations.txt groundtruth_file:=groundtruth.txt odometry_file:=dvo_odometry_${DATASET_NAME}.txt camera_matrix_file:=camera.txt scale:=1000 > ${LOGS_DIR}/dvo_log_${DATASET_NAME}.txt
	    # echo " done"
	    echo -n "Running NDT..."
	    rosrun ndt_evaluate ndt_eth_kinect_evaluate_node ${CONF_DIR}/ndt_eth_kinect.conf ${f}/associations.txt ${RESULTS_DIR}/ndt_odometry_${DATASET_NAME}.txt > ${LOGS_DIR}/ndt_log_${DATASET_NAME}.txt
	    echo " done"
	    # echo -n "Running KINFU..."
	    # optirun ${BIN_DIR}/kinfu_eth_kinect_evaluate ${CONF_DIR}/kinfu_eth_kinect.conf associations.txt ${RESULTS_DIR}/kinfu_odometry_${DATASET_NAME}.txt 0 > ${LOGS_DIR}/kinfu_log_${DATASET_NAME}.txt
	    # echo " done"
	    echo -n "Running NICP..."
	    ${BIN_DIR}/nicp_eth_kinect_evaluate ${CONF_DIR}/nicp_eth_kinect.conf associations.txt ${RESULTS_DIR}/nicp_odometry_${DATASET_NAME}.txt 0 > ${LOGS_DIR}/nicp_log_${DATASET_NAME}.txt
	    echo " done"
	    echo -n "Running NICP INCREMENTAL..."
	    ${BIN_DIR}/nicp_incremental_eth_kinect_evaluate ${CONF_DIR}/nicp_eth_kinect_incremental.conf associations.txt ${RESULTS_DIR}/nicp_incremental_odometry_${DATASET_NAME}.txt 0 > ${LOGS_DIR}/nicp_incremental_log_${DATASET_NAME}.txt
	    echo " done"
	fi

	##################################################################### 
	##                  Compute Relative Pose Errors                   ##
	#####################################################################
	echo -n "Computing RPE..."
	python ${SCRIPT_DIR}/evaluate_rpe.py --fixed_delta --plot ${RESULTS_DIR}/gicp_rpe_${DATASET_NAME}.png --verbose groundtruth.txt ${RESULTS_DIR}/gicp_odometry_${DATASET_NAME}.txt > ${RESULTS_DIR}/gicp_rpe_${DATASET_NAME}.txt
	# python ${SCRIPT_DIR}/evaluate_rpe.py --fixed_delta --plot ${RESULTS_DIR}/dvo_rpe_${DATASET_NAME}.png --verbose groundtruth.txt ${RESULTS_DIR}/dvo_odometry_${DATASET_NAME}.txt > ${RESULTS_DIR}/dvo_rpe_${DATASET_NAME}.txt
	python ${SCRIPT_DIR}/evaluate_rpe.py --fixed_delta --plot ${RESULTS_DIR}/ndt_rpe_${DATASET_NAME}.png --verbose groundtruth.txt ${RESULTS_DIR}/ndt_odometry_${DATASET_NAME}.txt > ${RESULTS_DIR}/ndt_rpe_${DATASET_NAME}.txt
	# python ${SCRIPT_DIR}/evaluate_rpe.py --fixed_delta --plot ${RESULTS_DIR}/kinfu_rpe_${DATASET_NAME}.png --verbose groundtruth.txt ${RESULTS_DIR}/kinfu_odometry_${DATASET_NAME}.txt > ${RESULTS_DIR}/kinfu_rpe_${DATASET_NAME}.txt
	python ${SCRIPT_DIR}/evaluate_rpe.py --fixed_delta --plot ${RESULTS_DIR}/nicp_rpe_${DATASET_NAME}.png --verbose groundtruth.txt ${RESULTS_DIR}/nicp_odometry_${DATASET_NAME}.txt > ${RESULTS_DIR}/nicp_rpe_${DATASET_NAME}.txt
	python ${SCRIPT_DIR}/evaluate_rpe.py --fixed_delta --plot ${RESULTS_DIR}/nicp_incremental_rpe_${DATASET_NAME}.png --verbose groundtruth.txt ${RESULTS_DIR}/nicp_incremental_odometry_${DATASET_NAME}.txt > ${RESULTS_DIR}/nicp_incremental_rpe_${DATASET_NAME}.txt
	echo " done"

	##################################################################### 
	##               Compute Absolute Trajectory Errors                ##
	#####################################################################
	echo -n "Computing ATE..."
	python ${SCRIPT_DIR}/evaluate_ate.py groundtruth.txt ${RESULTS_DIR}/gicp_odometry_${DATASET_NAME}.txt --plot ${RESULTS_DIR}/gicp_ate_${DATASET_NAME}.png --verbose > ${RESULTS_DIR}/gicp_ate_${DATASET_NAME}.txt
	# python ${SCRIPT_DIR}/evaluate_ate.py groundtruth.txt ${RESULTS_DIR}/dvo_odometry_${DATASET_NAME}.txt --plot ${RESULTS_DIR}/dvo_ate_${DATASET_NAME}.png --verbose > ${RESULTS_DIR}/dvo_ate_${DATASET_NAME}.txt
	python ${SCRIPT_DIR}/evaluate_ate.py groundtruth.txt ${RESULTS_DIR}/ndt_odometry_${DATASET_NAME}.txt --plot ${RESULTS_DIR}/ndt_ate_${DATASET_NAME}.png --verbose > ${RESULTS_DIR}/ndt_ate_${DATASET_NAME}.txt
	# python ${SCRIPT_DIR}/evaluate_ate.py groundtruth.txt ${RESULTS_DIR}/kinfu_odometry_${DATASET_NAME}.txt --plot ${RESULTS_DIR}/kinfu_ate_${DATASET_NAME}.png --verbose > ${RESULTS_DIR}/kinfu_ate_${DATASET_NAME}.txt
	python ${SCRIPT_DIR}/evaluate_ate.py groundtruth.txt ${RESULTS_DIR}/nicp_odometry_${DATASET_NAME}.txt --plot ${RESULTS_DIR}/nicp_ate_${DATASET_NAME}.png --verbose > ${RESULTS_DIR}/nicp_ate_${DATASET_NAME}.txt
	python ${SCRIPT_DIR}/evaluate_ate.py groundtruth.txt ${RESULTS_DIR}/nicp_incremental_odometry_${DATASET_NAME}.txt --plot ${RESULTS_DIR}/nicp_incremental_ate_${DATASET_NAME}.png --verbose > ${RESULTS_DIR}/nicp_incremental_ate_${DATASET_NAME}.txt
	echo " done"
	
	cd ${CURRENT_DIR}

	##################################################################### 
	##                   Extract Computation Time                      ##
	#####################################################################
	echo -n "Extracting computation times..."
	echo -n	"${DATASET_NAME} " >> ${RESULTS_DIR}/computation_times.txt
	echo -n	"`grep "Mean time frame:" ${LOGS_DIR}/gicp_log_${DATASET_NAME}.txt | awk '{ print $4 }'` " >> ${RESULTS_DIR}/computation_times.txt
	echo -n	"`grep "Mean time frame:" ${LOGS_DIR}/dvo_log_${DATASET_NAME}.txt | awk '{ print $4 }'` " >> ${RESULTS_DIR}/computation_times.txt
	echo -n	"`grep "Mean time frame:" ${LOGS_DIR}/ndt_log_${DATASET_NAME}.txt | awk '{ print $4 }'` " >> ${RESULTS_DIR}/computation_times.txt
	echo -n	"`grep "Mean time frame:" ${LOGS_DIR}/kinfu_log_${DATASET_NAME}.txt | awk '{ print $4 }'` " >> ${RESULTS_DIR}/computation_times.txt
	echo -n "`grep "Mean time frame:" ${LOGS_DIR}/nicp_log_${DATASET_NAME}.txt | awk '{ print $4 }'` " >> ${RESULTS_DIR}/computation_times.txt	
	echo "`grep "Mean time frame:" ${LOGS_DIR}/nicp_incremental_log_${DATASET_NAME}.txt | awk '{ print $4 }'` " >> ${RESULTS_DIR}/computation_times.txt	
	echo " done"

	##################################################################### 
	##                  Extract Relative Pose Errors                   ##
	#####################################################################
	echo -n "Extracting RPE errors..."
	echo -n	"${DATASET_NAME} " >> ${RESULTS_DIR}/rpe_translational_means.txt
	echo -n	"`grep "translational_error.mean" ${RESULTS_DIR}/gicp_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_means.txt
	echo -n	"`grep "translational_error.mean" ${RESULTS_DIR}/dvo_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_means.txt
	echo -n	"`grep "translational_error.mean" ${RESULTS_DIR}/ndt_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_means.txt
	echo -n	"`grep "translational_error.mean" ${RESULTS_DIR}/kinfu_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_means.txt
	echo -n "`grep "translational_error.mean" ${RESULTS_DIR}/nicp_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_means.txt	
	echo "`grep "translational_error.mean" ${RESULTS_DIR}/nicp_incremental_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_means.txt	

	echo -n	"${DATASET_NAME} " >> ${RESULTS_DIR}/rpe_translational_medians.txt
	echo -n	"`grep "translational_error.median" ${RESULTS_DIR}/gicp_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_medians.txt
	echo -n	"`grep "translational_error.median" ${RESULTS_DIR}/dvo_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_medians.txt
	echo -n	"`grep "translational_error.median" ${RESULTS_DIR}/ndt_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_medians.txt
	echo -n	"`grep "translational_error.median" ${RESULTS_DIR}/kinfu_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_medians.txt
	echo -n "`grep "translational_error.median" ${RESULTS_DIR}/nicp_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_medians.txt	
	echo "`grep "translational_error.median" ${RESULTS_DIR}/nicp_incremental_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_medians.txt	

	echo -n	"${DATASET_NAME} " >> ${RESULTS_DIR}/rpe_translational_stds.txt
	echo -n	"`grep "translational_error.std" ${RESULTS_DIR}/gicp_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_stds.txt
	echo -n	"`grep "translational_error.std" ${RESULTS_DIR}/dvo_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_stds.txt
	echo -n	"`grep "translational_error.std" ${RESULTS_DIR}/ndt_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_stds.txt
	echo -n	"`grep "translational_error.std" ${RESULTS_DIR}/kinfu_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_stds.txt
	echo -n "`grep "translational_error.std" ${RESULTS_DIR}/nicp_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_stds.txt
	echo "`grep "translational_error.std" ${RESULTS_DIR}/nicp_incremental_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_translational_stds.txt

	echo -n	"${DATASET_NAME} " >> ${RESULTS_DIR}/rpe_rotational_means.txt
	echo -n	"`grep "rotational_error.mean" ${RESULTS_DIR}/gicp_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_means.txt
	echo -n	"`grep "rotational_error.mean" ${RESULTS_DIR}/dvo_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_means.txt
	echo -n	"`grep "rotational_error.mean" ${RESULTS_DIR}/ndt_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_means.txt
	echo -n	"`grep "rotational_error.mean" ${RESULTS_DIR}/kinfu_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_means.txt
	echo -n	"`grep "rotational_error.mean" ${RESULTS_DIR}/nicp_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_means.txt	
	echo "`grep "rotational_error.mean" ${RESULTS_DIR}/nicp_incremental_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_means.txt	

	echo -n	"${DATASET_NAME} " >> ${RESULTS_DIR}/rpe_rotational_medians.txt
	echo -n	"`grep "rotational_error.median" ${RESULTS_DIR}/gicp_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_medians.txt
	echo -n	"`grep "rotational_error.median" ${RESULTS_DIR}/dvo_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_medians.txt
	echo -n	"`grep "rotational_error.median" ${RESULTS_DIR}/ndt_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_medians.txt
	echo -n	"`grep "rotational_error.median" ${RESULTS_DIR}/kinfu_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_medians.txt
	echo -n	"`grep "rotational_error.median" ${RESULTS_DIR}/nicp_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_medians.txt	
	echo "`grep "rotational_error.median" ${RESULTS_DIR}/nicp_incremental_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_medians.txt	

	echo -n	"${DATASET_NAME} " >> ${RESULTS_DIR}/rpe_rotational_stds.txt
	echo -n	"`grep "rotational_error.std" ${RESULTS_DIR}/gicp_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_stds.txt
	echo -n	"`grep "rotational_error.std" ${RESULTS_DIR}/dvo_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_stds.txt
	echo -n	"`grep "rotational_error.std" ${RESULTS_DIR}/ndt_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_stds.txt
	echo -n	"`grep "rotational_error.std" ${RESULTS_DIR}/kinfu_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_stds.txt
	echo -n "`grep "rotational_error.std" ${RESULTS_DIR}/nicp_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_stds.txt	
	echo "`grep "rotational_error.std" ${RESULTS_DIR}/nicp_incremental_rpe_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/rpe_rotational_stds.txt	
	echo " done"

	##################################################################### 
	##               Extract Absolute Trajectory Error                 ##
	#####################################################################
	echo -n "Extracting ATE errors..."
	echo -n	"${DATASET_NAME} " >> ${RESULTS_DIR}/ate_translational_means.txt
	echo -n	"`grep "absolute_translational_error.mean" ${RESULTS_DIR}/gicp_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_means.txt
	echo -n	"`grep "absolute_translational_error.mean" ${RESULTS_DIR}/dvo_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_means.txt
	echo -n	"`grep "absolute_translational_error.mean" ${RESULTS_DIR}/ndt_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_means.txt
	echo -n	"`grep "absolute_translational_error.mean" ${RESULTS_DIR}/kinfu_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_means.txt
	echo -n	"`grep "absolute_translational_error.mean" ${RESULTS_DIR}/nicp_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_means.txt	
	echo "`grep "absolute_translational_error.mean" ${RESULTS_DIR}/nicp_incremental_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_means.txt	

	echo -n	"${DATASET_NAME} " >> ${RESULTS_DIR}/ate_translational_medians.txt
	echo -n	"`grep "absolute_translational_error.median" ${RESULTS_DIR}/gicp_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_medians.txt
	echo -n	"`grep "absolute_translational_error.median" ${RESULTS_DIR}/dvo_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_medians.txt
	echo -n	"`grep "absolute_translational_error.median" ${RESULTS_DIR}/ndt_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_medians.txt
	echo -n	"`grep "absolute_translational_error.median" ${RESULTS_DIR}/kinfu_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_medians.txt
	echo -n	"`grep "absolute_translational_error.median" ${RESULTS_DIR}/nicp_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_medians.txt	
	echo "`grep "absolute_translational_error.median" ${RESULTS_DIR}/nicp_incremental_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_medians.txt	

	echo -n	"${DATASET_NAME} " >> ${RESULTS_DIR}/ate_translational_stds.txt
	echo -n	"`grep "absolute_translational_error.std" ${RESULTS_DIR}/gicp_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_stds.txt
	echo -n	"`grep "absolute_translational_error.std" ${RESULTS_DIR}/dvo_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_stds.txt
	echo -n	"`grep "absolute_translational_error.std" ${RESULTS_DIR}/ndt_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_stds.txt
	echo -n	"`grep "absolute_translational_error.std" ${RESULTS_DIR}/kinfu_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_stds.txt
	echo -n	"`grep "absolute_translational_error.std" ${RESULTS_DIR}/nicp_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_stds.txt
	echo "`grep "absolute_translational_error.std" ${RESULTS_DIR}/nicp_incremental_ate_${DATASET_NAME}.txt | awk '{ print $2 }'` " >> ${RESULTS_DIR}/ate_translational_stds.txt
	echo " done"
    fi
done;

##################################################################### 
##               Compute mean computation times                    ##
#####################################################################
echo -n "Computing mean computation times..."
echo -n	"Mean Computation Time " >> ${RESULTS_DIR}/mean_computation_times.txt
echo -n "`awk '{ sum += $2; n++ } END { if (n > 0) print sum / n; }' results/computation_times.txt` " >> ${RESULTS_DIR}/mean_computation_times.txt
echo -n "`awk '{ sum += $3; n++ } END { if (n > 0) print sum / n; }' results/computation_times.txt` " >> ${RESULTS_DIR}/mean_computation_times.txt
echo -n "`awk '{ sum += $4; n++ } END { if (n > 0) print sum / n; }' results/computation_times.txt` " >> ${RESULTS_DIR}/mean_computation_times.txt
echo -n "`awk '{ sum += $5; n++ } END { if (n > 0) print sum / n; }' results/computation_times.txt` " >> ${RESULTS_DIR}/mean_computation_times.txt
echo -n "`awk '{ sum += $6; n++ } END { if (n > 0) print sum / n; }' results/computation_times.txt` " >> ${RESULTS_DIR}/mean_computation_times.txt
echo -n "`awk '{ sum += $7; n++ } END { if (n > 0) print sum / n; }' results/computation_times.txt` " >> ${RESULTS_DIR}/mean_computation_times.txt
echo " done"

killall roscore

echo "Translational means"
cat ${RESULTS_DIR}/rpe_translational_means.txt
echo "Rotational means"
cat ${RESULTS_DIR}/rpe_rotational_means.txt
