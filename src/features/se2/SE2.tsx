import * as log from "loglevel";
import { useState, CSSProperties, useRef, useEffect } from "react";
import { useAppDispatch, useAppSelector } from "../../app/hooks";
import { Box, ButtonGroup, FormControl, FormControlLabel, FormLabel, Radio, RadioGroup, Stack } from "@mui/material";
import { SE2Types } from "./se2.utils";
import { setPose, selectPose } from "./se2Slice";

import TargetAnchor from "./TargetAnchor";

// from https://ieeexplore.ieee.org/document/9636008
function SE2(
    {
        width, height, scale = 1, interfaceType, widthInputType,
        color = { default: "black", accent: "grey" }, style,
        background
    }: {
        width: number,
        height: number,
        scale?: number,
        interfaceType: 'targetanchor',
        widthInputType: 'none' | 'buttons' | 'drag'
        color?: {
            default?: string,
            accent?: string,
            axes?: {
                x?: string,
                y?: string,
                r?: string
            }
        }
        style?: CSSProperties
        background?: JSX.Element
    }) {
    const dispatch = useAppDispatch();
    const pose = useAppSelector(selectPose);

    const [localPose, setLocalPose] = useState<SE2Types.Pose>({ x: -1, y: -1, theta: 0, width: 0 })
    const [mousePressed, setMousePressed] = useState(false);
    const [poseWidth, setPoseWidth] = useState(40);

    const onMouseDown = (event) => {
        const { top, left } = event.target.getBoundingClientRect();

        const x = event.clientX - left;
        const y = event.clientY - top;
        log.debug("Click at", x, y);
        if (widthInputType === "buttons") {
            setLocalPose({
                x: x - poseWidth / 2,
                y: y,
                theta: 0,
                width: 0
            })
        } else {
            setLocalPose({
                x: x,
                y: y,
                theta: 0,
                width: 0
            })
        }

        setMousePressed(true)
    }

    const onMouseMove = (event) => {
        if (mousePressed) {
            const { top, left } = event.target.getBoundingClientRect();

            const x = event.clientX - left;
            const y = event.clientY - top;

            if (widthInputType == 'drag') {
                setPoseWidth(Math.sqrt(Math.pow(x - localPose.x, 2) + Math.pow(y - localPose.y, 2)));
            }

            const theta = -Math.atan2(y - localPose.y, x - localPose.x);

            if (widthInputType === "buttons") {
                setLocalPose({
                    x: localPose.x + (Math.cos(localPose.theta) - Math.cos(theta)) * poseWidth/2,
                    y: localPose.y + (Math.sin(localPose.theta) - Math.sin(theta)) * poseWidth/2,
                    theta: theta,
                    width: poseWidth
                })
            } else {
                setLocalPose({
                    x: localPose.x,
                    y: localPose.y,
                    theta: theta,
                    width: 0
                })
            }
        }
    }

    const onMouseUp = (event) => {
        setMousePressed(false)
        dispatch(setPose(localPose))
        log.debug(localPose)
    }

    const onMouseLeave = (event) => {
        setMousePressed(false)
    }

    useEffect(() => {
        if (!mousePressed) {
            setLocalPose(pose);
        }
    }, [pose])

    return <Stack direction='column' spacing={2}>
        <Box sx={{ position: "relative", width: width, height: height }}>
            <svg viewBox={`0 0 ${width / scale} ${height / scale}`} width={width} height={height} style={style}
                onMouseDown={onMouseDown} onMouseMove={onMouseMove} onMouseUp={onMouseUp} onMouseLeave={onMouseLeave}>
                {interfaceType == "targetanchor" &&
                    <TargetAnchor
                        x={(widthInputType === "buttons" ? localPose.x + Math.cos(localPose.theta) * poseWidth/2 : localPose.x) / scale}
                        y={(widthInputType === "buttons" ? localPose.y + Math.sin(localPose.theta) * poseWidth/2 : localPose.y) / scale}
                        theta={- localPose.theta * 180 / Math.PI}
                        color={mousePressed ? color.accent : color.default}
                        width={poseWidth / scale}
                        centered={widthInputType === "buttons"}
                    />
                }
            </svg>
            {background}
        </Box>
        <Box>
            <FormLabel id="demo-controlled-radio-buttons-group">Gripper Width</FormLabel>
            <RadioGroup
                aria-labelledby="demo-controlled-radio-buttons-group"
                name="controlled-radio-buttons-group"
                row
                value={poseWidth}
                onChange={(event: React.ChangeEvent<HTMLInputElement>) => {
                    setPoseWidth(parseInt((event.target as HTMLInputElement).value))
                }}
            >
                <FormControlLabel value={40} control={<Radio />} label="Small" />
                <FormControlLabel value={100} control={<Radio />} label="Medium" />
                <FormControlLabel value={200} control={<Radio />} label="Large" />
            </RadioGroup>
        </Box>
    </Stack>


}

export default SE2;