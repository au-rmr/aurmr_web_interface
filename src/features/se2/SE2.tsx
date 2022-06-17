import * as log from "loglevel";
import { useState, CSSProperties, useRef, useEffect } from "react";
import { useAppDispatch, useAppSelector } from "../../app/hooks";
import { setPose, selectPose } from "./se2Slice";

import TargetAnchor from "./TargetAnchor";

// from https://ieeexplore.ieee.org/document/9636008
function SE2(
    {
        width, height, scale = 1, interfaceType,
        color = { default: "black", accent: "grey" }, style
    }: {
        width: number,
        height: number,
        scale?: number,
        interfaceType: 'targetanchor',
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
    }) {
    const dispatch = useAppDispatch();
    const pose = useAppSelector(selectPose);

    const [localPose, setLocalPose] = useState({ x: -1, y: -1, theta: 0 })
    const [mousePressed, setMousePressed] = useState(false);

    const onMouseDown = (event) => {
        const { top, left } = event.target.getBoundingClientRect();

        const x = event.clientX - left;
        const y = event.clientY - top;
        log.debug("Click at", x, y);
        setLocalPose({
            x: x,
            y: y,
            theta: 0
        })

        setMousePressed(true)
    }

    const onMouseMove = (event) => {
        if (mousePressed) {
            const { top, left } = event.target.getBoundingClientRect();
    
            const x = event.clientX - left;
            const y = event.clientY - top;

            setLocalPose({
                x: localPose.x,
                y: localPose.y,
                theta: Math.atan2(y - localPose.y, x - localPose.x) * 180 / Math.PI
            })
        }
    }

    const onMouseUp = (event) => {
        setMousePressed(false)
        dispatch(setPose(localPose))
    }

    const onMouseLeave = (event) => {
        setMousePressed(false)
    }

    useEffect(() => {
        if (!mousePressed) {
            setLocalPose(pose);
        }
    }, [pose])

    return <svg viewBox={`0 0 ${width/scale} ${height/scale}`} width={width} height={height} style={style}
        onMouseDown={onMouseDown} onMouseMove={onMouseMove} onMouseUp={onMouseUp} onMouseLeave={onMouseLeave}>
        {interfaceType == "targetanchor" &&
            <TargetAnchor x={localPose.x/scale} y={localPose.y/scale} theta={localPose.theta} color={mousePressed ? color.accent : color.default} />
        }
    </svg>

}

export default SE2;