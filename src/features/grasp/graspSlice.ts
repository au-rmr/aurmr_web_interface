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
            generateHeuristicGraspService.unadvertise()
            // resolve({
            //     width: result.grasp.width,
            //     pose: {
            //         position: {
            //             x: result.grasp.pose.position.x,
            //             y: result.grasp.pose.position.y,
            //             z: result.grasp.pose.position.z
            //         },
            //         orientation: {
            //             x: result.grasp.pose.orientation.x,
            //             y: result.grasp.pose.orientation.y,
            //             z: result.grasp.pose.orientation.z,
            //             w: result.grasp.pose.orientation.w
            //         }
            //     }
            // })
            // resolve(JSON.parse(JSON.stringify(result.grasp)))
            resolve(result.grasp)
        });
        } else {
            reject("invalid pose")
        }
    })
})

export interface Grasp {
    width: number,
    pose: {
        position: {
            x: number,
            y: number,
            z: number
        },
        orientation: {
            x: number,
            y: number,
            z: number,
            w: number
        }
    }
}

export interface GraspState {
    graspGeneration: {
        status: 'idle' | 'loading' ,
        generatedGrasp?: Grasp
    }
}

const initialState: GraspState = {
    graspGeneration: {
        status: "idle",
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
                state.graspGeneration.generatedGrasp = action.payload
                state.graspGeneration.status = 'idle'
            })
    }
})

// export const { disconnect } = rosSlice.actions;

export const selectRequestingGrasp = (state: AppState) => state.grasp.graspGeneration.status;

export default graspSlice.reducer;