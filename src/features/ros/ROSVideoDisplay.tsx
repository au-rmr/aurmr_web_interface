import {
    Box,
    Button,
    Paper,
    Stack,
    SxProps,
    TextField,
    Theme,
    Typography,
} from "@mui/material";

import * as log from "loglevel";
import { CSSProperties, useEffect, useState } from "react";

import { useAppSelector } from "../../app/hooks";
import { selectConnected, ROS } from "./rosSlice";
import * as ROSLIB from 'roslib';
import { ROSTypes } from "./ros.utils";

function ROSVideoDisplay({ topicName, style = {} }: { topicName: string, style?: CSSProperties }) {
    const connected = useAppSelector(selectConnected);

    const [imgString, setImgString] = useState('');


    useEffect(() => {
        if (connected) {
            const imageTopic = new ROSLIB.Topic<ROSTypes.CompressedImage>({
                ros: ROS,
                name: topicName,
                messageType: 'sensor_msgs/CompressedImage'

            });
            imageTopic.subscribe((message => {
                setImgString('data:image/jpg;base64,' + message.data)
            }));
        }
    }, [connected])

    return (
        <Box>
            {connected &&
                <img src={imgString} style={style} />
            }
        </Box>
    );
}

export default ROSVideoDisplay;
