import type { NextPage } from "next";
import Head from "next/head";

import { Box, Link, Stack, Typography } from "@mui/material";

import Counter from "../features/counter/Counter";
import ROSConnection from "../features/ros/ROSConnection";
import ROSVideoDisplay from "../features/ros/ROSVideoDisplay";

import * as log from 'loglevel'
import SE2 from "../features/se2/SE2";
import { useState } from "react";

log.setLevel(log.levels.TRACE);

const IndexPage: NextPage = () => {
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

  return (
    <div>
      <Head>
        <title>Redux Toolkit</title>
        <link rel="icon" href="/favicon.ico" />
      </Head>
      <Stack spacing={2} alignItems="center" mt={2}>
        {/* <Counter /> */}
        <Box sx={{ position: "relative", width: videoStreamSize.x, height: videoStreamSize.y }}>
          <SE2 width={videoStreamSize.x} height={videoStreamSize.y} interfaceType="targetanchor" style={{ border: 'solid', borderRadius: 4, position: "absolute", top: 0, left: 0, zIndex: 10 }} />
          <ROSVideoDisplay style={{ borderRadius: 4, width: videoStreamSize.x, position: "absolute", top: 0, left: 0 }} topicName="/camera_lower_right/color/image_raw/compressed" streamSizeCallback={handleStreamSize} />
        </Box>
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
