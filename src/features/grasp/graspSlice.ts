import { createSlice, createAsyncThunk } from '@reduxjs/toolkit'

import * as ROSLIB from 'roslib';
import store, { AppState } from '../../app/store';
import * as log from 'loglevel';

import { ROS } from '../ros/rosSlice';
import { SE2Types } from '../se2/se2.utils';

export const generateHeuristicGrasp = createAsyncThunk<Grasp, SE2Types.Pose>('grasp/generateHeuristicGrasp', (pose: SE2Types.Pose) => {
    log.debug("Generating Heuristic Grasp")
    log.debug(pose)

    const generateHeuristicGraspService = new ROSLIB.Service({
        ros: ROS,
        name: "/generate_heuristic_grasp",
        serviceType: "aurmr_web_interface/GenerateHeuristicGrasp"
    })

    const request = new ROSLIB.ServiceRequest({
        se2: pose
    })

    return new Promise((resolve, reject) => {
        if (pose.x > 0 && pose.y > 0) {
            generateHeuristicGraspService.callService(request, function (result) {
                log.debug('Result for service call on '
                    + generateHeuristicGrasp.name
                    + ': '
                    + result);
                resolve({
                    width: result.grasp.width,
                    rotation: result.grasp.rotation,
                    center: {
                        x: result.grasp.center.x,
                        y: result.grasp.center.y,
                        z: result.grasp.center.z
                    }
                })
            });
            generateHeuristicGraspService.unadvertise()
        } else {
            reject("invalid pose")
        }
    })
})

export interface Grasp {
    width: number,
    rotation: number,
    center: {
        x: number,
        y: number,
        z: number
    }
}

export interface GraspState {
    graspGeneration: {
        status: string,
        generatedGrasp?: Grasp
    }
}

const initialState: GraspState = {
    graspGeneration: {
        status: "n/a",
        generatedGrasp: null
    }
}

export const graspSlice = createSlice({
    name: 'grasp',
    initialState,
    reducers: {
    },
    extraReducers: builder => {
        builder
            .addCase(generateHeuristicGrasp.pending, (state, action) => {
                state.graspGeneration.status = 'loading'
            })
            .addCase(generateHeuristicGrasp.fulfilled, (state, action) => {
                log.debug(action.payload)
                state.graspGeneration.generatedGrasp = action.payload
                state.graspGeneration.status = 'idle'
            })
    }
})

// export const { disconnect } = rosSlice.actions;

// export const selectConnected = (state: AppState) => state.ros.connected;

export default graspSlice.reducer;