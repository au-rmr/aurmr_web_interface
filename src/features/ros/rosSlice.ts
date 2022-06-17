import {createSlice, PayloadAction } from '@reduxjs/toolkit'

import * as ROSLIB from 'roslib';
import store, { AppState } from '../../app/store';
import * as log from 'loglevel';

export interface RosState {
    connected: boolean
}

const initialState: RosState = {
    connected: false
}

export const rosSlice = createSlice({
    name: 'ros',
    initialState,
    reducers: {
        connect: state => {
            state.connected = true;
        },
        disconnect: state => {
            state.connected = false;
        }
    }
})

export const { disconnect } = rosSlice.actions;

export const selectConnected = (state: AppState) => state.ros.connected;

export const ROS = new ROSLIB.Ros({url : 'ws://localhost:9090'})

ROS.on('connection', () => {
    log.debug("ROS Connected");
    store.dispatch(rosSlice.actions.connect());
})
ROS.on('error', (event: any) => {
    log.error("ROS Error: " + event)
    store.dispatch(rosSlice.actions.disconnect());

})
ROS.on('close', () => {
    log.debug("ROS Disconnected")
    store.dispatch(rosSlice.actions.disconnect());
})

export default rosSlice.reducer;