import type { NextPage } from "next";
import Head from "next/head";

import { Box, Link, Stack, Typography } from "@mui/material";

import Counter from "../features/counter/Counter";
import ROSConnection from "../features/ros/ROSConnection";
import ROSVideoDisplay from "../features/ros/ROSVideoDisplay";

import * as log from 'loglevel'
import SE2 from "../features/se2/SE2";
log.setLevel(log.levels.TRACE);

const IndexPage: NextPage = () => {
  return (
    <div>
      <Head>
        <title>Redux Toolkit</title>
        <link rel="icon" href="/favicon.ico" />
      </Head>
      <Stack spacing={2} alignItems="center" mt={2}>
        {/* <Counter /> */}
        <SE2 width={500} height={500} interfaceType="targetanchor" style={{ border: 'solid' }} />
        <Stack direction='row' spacing={2}>
          <ROSVideoDisplay style={{ borderRadius: 4, width: 500 }} topicName="/camera_lower_right/color/image_raw/compressed" />
          <ROSVideoDisplay style={{ borderRadius: 4, width: 500 }} topicName="/camera_lower_left/color/image_raw/compressed" />
        </Stack>
        <ROSConnection />
        <Typography>
          Edit <code>src/App.tsx</code> and save to reload.
        </Typography>
        <Box>
          <Typography>
            Learn{' '}
            <Link
              href="https://reactjs.org/"
              target="_blank"
              rel="noopener noreferrer"
            >
              React
            </Link>
            ,{' '}
            <Link
              href="https://redux.js.org/"
              target="_blank"
              rel="noopener noreferrer"
            >
              Redux
            </Link>
            ,{' '}
            <Link
              href="https://redux-toolkit.js.org/"
              target="_blank"
              rel="noopener noreferrer"
            >
              Redux Toolkit
            </Link>
            , and{' '}
            <Link
              href="https://react-redux.js.org/"
              target="_blank"
              rel="noopener noreferrer"
            >
              React Redux
            </Link>
          </Typography>
        </Box>
      </Stack>
    </div>
  );
};

export default IndexPage;
