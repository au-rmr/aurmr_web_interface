import {
  Box,
  Button,
  Paper,
  Stack,
  TextField,
  Typography,
} from "@mui/material";
import { useState } from "react";

import { useAppSelector, useAppDispatch } from "../../app/hooks";
import { selectConnected, disconnect } from "./rosSlice";

function ROSConnection() {
  const dispatch = useAppDispatch();
  const connected = useAppSelector(selectConnected);

  return (
    <Paper>
      <Stack
        direction="column"
        justifyContent="center"
        alignItems="center"
        spacing={2}
        m={2}
      >
        {connected
          ? <Stack direction="row" spacing={2} alignItems="center">
            <Typography>Connected</Typography>
            <Button variant="outlined" onClick={() => dispatch(disconnect())}>
              Disconnect
            </Button>
          </Stack>
          : <Typography>Disconnected</Typography>
        }
      </Stack>
    </Paper>
  );
}

export default ROSConnection;
