import type { NextPage } from 'next'
import Head from 'next/head'

import { Link } from '@mui/material'

import Counter from '../features/counter/Counter'

const IndexPage: NextPage = () => {
  return (
    <div>
      <Head>
        <title>Redux Toolkit</title>
        <link rel="icon" href="/favicon.ico" />
      </Head>
      <header>
        <Counter />
        <p>
          Edit <code>src/App.tsx</code> and save to reload.
        </p>
        <span>
          <span>Learn </span>
          <Link
            href="https://reactjs.org/"
            target="_blank"
            rel="noopener noreferrer"
          >
            React
          </Link>
          <span>, </span>
          <Link
            href="https://redux.js.org/"
            target="_blank"
            rel="noopener noreferrer"
          >
            Redux
          </Link>
          <span>, </span>
          <Link
            href="https://redux-toolkit.js.org/"
            target="_blank"
            rel="noopener noreferrer"
          >
            Redux Toolkit
          </Link>
          ,<span> and </span>
          <Link
            href="https://react-redux.js.org/"
            target="_blank"
            rel="noopener noreferrer"
          >
            React Redux
          </Link>
        </span>
      </header>
    </div>
  )
}

export default IndexPage
