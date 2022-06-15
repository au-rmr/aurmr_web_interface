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
import {
  decrement,
  increment,
  incrementByAmount,
  incrementAsync,
  incrementIfOdd,
  selectCount,
} from "./counterSlice";

function Counter() {
  const dispatch = useAppDispatch();
  const count = useAppSelector(selectCount);
  const [incrementAmount, setIncrementAmount] = useState("2");

  const incrementValue = Number(incrementAmount) || 0;

  return (
    <Paper>
      <Stack
        direction="column"
        justifyContent="center"
        alignItems="center"
        spacing={2}
        m={2}
      >
          <Stack direction="row" spacing={2} alignItems="center" mb={5}>
            <Button
              variant="contained"
              size="small"
              aria-label="Decrement value"
              onClick={() => dispatch(decrement())}
            >
              -
            </Button>
            <Typography>{count}</Typography>
            <Button
              variant="contained"
              size="small"
              aria-label="Increment value"
              onClick={() => dispatch(increment())}
            >
              +
            </Button>
          </Stack>
        <Stack direction="row" spacing={2}>
          <TextField
            aria-label="Set increment amount"
            type="number"
            value={incrementAmount}
            onChange={(e) => setIncrementAmount(e.target.value)}
          />
          <Button
            variant="outlined"
            onClick={() => dispatch(incrementByAmount(incrementValue))}
          >
            Add Amount
          </Button>
          <Button
            variant="outlined"
            onClick={() => dispatch(incrementAsync(incrementValue))}
          >
            Add Async
          </Button>
          <Button
            variant="outlined"
            onClick={() => dispatch(incrementIfOdd(incrementValue))}
          >
            Add If Odd
          </Button>
        </Stack>
      </Stack>
    </Paper>
  );
}

export default Counter;
