import { createSlice, PayloadAction } from "@reduxjs/toolkit";
import * as log from 'loglevel';
import { AppState } from "../../app/store";
import { SE2Types } from "./se2.utils";

const initialState: SE2Types.Pose = {
    x: -1,
    y: -1,
    theta: 0,
    width: 0
}

export const se2Slice = createSlice({
    name: 'se2',
    initialState,
    reducers: {
        setPose: (state, action: PayloadAction<SE2Types.Pose>) => {
            state.x = action.payload.x;
            state.y = action.payload.y;
            state.theta = action.payload.theta;
            state.width = action.payload.width;
        }
    }
})

export const { setPose } = se2Slice.actions;
export const selectPose = (state: AppState) => state.se2

export default se2Slice.reducer