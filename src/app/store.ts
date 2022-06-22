import { configureStore, ThunkAction, Action } from '@reduxjs/toolkit'

import counterReducer from '../features/counter/counterSlice'
import rosReducer from '../features/ros/rosSlice';
import se2Reducer from '../features/se2/se2Slice';
import graspReducer from '../features/grasp/graspSlice';

export function makeStore() {
  return configureStore({
    reducer: { 
      counter: counterReducer,
      ros: rosReducer,
      se2: se2Reducer,
      grasp: graspReducer
    },
  })
}

const store = makeStore()

export type AppState = ReturnType<typeof store.getState>

export type AppDispatch = typeof store.dispatch

export type AppThunk<ReturnType = void> = ThunkAction<
  ReturnType,
  AppState,
  unknown,
  Action<string>
>

export default store
