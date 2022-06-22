import type { NextPage } from "next";
import Head from "next/head";

import { Box, Button, Link, Paper, Stack, Typography } from "@mui/material";

import ROSConnection from "../features/ros/ROSConnection";
import ROSVideoDisplay from "../features/ros/ROSVideoDisplay";

import * as log from 'loglevel'
import SE2 from "../features/se2/SE2";
import { useState } from "react";
import store from '../app/store';
import { generateHeuristicGrasp } from "../features/grasp/graspSlice";
import { useAppDispatch } from "../app/hooks";

log.setLevel(log.levels.TRACE);

const IndexPage: NextPage = () => {
  const dispatch = useAppDispatch();

  const [videoStreamSize, setVideoStreamSize] = useState({ x: 1920, y: 1080 });
  const targetWidth = 750;
  const handleStreamSize = (size: { x: number, y: number }) => {
    const s = { x: targetWidth, y: size.y };
    s.x = targetWidth;
    const ratio = size.x / targetWidth;
    s.y /= ratio;
    setVideoStreamSize(s)
    log.debug("Stream Size", videoStreamSize)
  }

  const handleGenerateGrasp = () => {
    dispatch(generateHeuristicGrasp(store.getState().se2))
  };

  return (
    <div>
      <Head>
        <title>Redux Toolkit</title>
        <link rel="icon" href="/favicon.ico" />
      </Head>
      <Stack spacing={2} alignItems="center" mt={2}>
        {/* <Counter /> */}
        <Stack spacing={2} direction="row" alignItems="center">
          <Box sx={{ position: "relative", width: videoStreamSize.x, height: videoStreamSize.y }}>
            <SE2 width={videoStreamSize.x} height={videoStreamSize.y} interfaceType="targetanchor" style={{ border: 'solid', borderRadius: 4, position: "absolute", top: 0, left: 0, zIndex: 10 }} />
            <ROSVideoDisplay style={{ borderRadius: 4, width: videoStreamSize.x, position: "absolute", top: 0, left: 0 }} topicName="/camera_wrist/color/image_raw/compressed" streamSizeCallback={handleStreamSize} />
          </Box>
          <Paper>
            <Stack spacing={2} sx={{m: 2}}>
              <Button variant="outlined"
              onClick={handleGenerateGrasp}>
                Generate Grasp
              </Button>
              <Button variant="contained">
                Execute Grasp
              </Button>
            </Stack>
          </Paper>
        </Stack>
        <Stack direction='row' spacing={2}>
          <ROSVideoDisplay style={{ borderRadius: 4, width: 500 }} topicName="/camera_lower_right/color/image_raw/compressed" />
          <ROSVideoDisplay style={{ borderRadius: 4, width: 500 }} topicName="/camera_lower_left/color/image_raw/compressed" />
        </Stack>
        <ROSConnection />
      </Stack>
    </div>
  );
};

export default IndexPage;
