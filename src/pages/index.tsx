import type { NextPage } from "next";
import Head from "next/head";

import { Box, Link, Stack, Typography } from "@mui/material";

import Counter from "../features/counter/Counter";

const IndexPage: NextPage = () => {
  return (
    <div>
      <Head>
        <title>Redux Toolkit</title>
        <link rel="icon" href="/favicon.ico" />
      </Head>
      <Stack spacing={2} alignItems="center" mt={2}>
        <Counter />
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
