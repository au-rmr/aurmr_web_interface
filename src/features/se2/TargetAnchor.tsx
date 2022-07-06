import { SE2Types } from './se2.utils'

function TargetAnchor({x, y, theta, width, color}: SE2Types.Pose & {color: string}) {
    return <g transform={`translate(${x} ${y}) rotate(${theta})`}>
        <line x1={0} y1={0} x2={width} y2={0} stroke={color} strokeLinecap="round" strokeWidth={7}/>
        <circle cx={0} cy={0} r={7} fill={color}/>
    </g>
}

export default TargetAnchor;