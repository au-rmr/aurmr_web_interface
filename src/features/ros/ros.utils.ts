import * as ROSLIB from 'roslib';

export namespace ROSTypes {
    export interface CompressedImage extends ROSLIB.Message {
        header: string,
        format: "jpeg" | "png",
        data: string
    }
}