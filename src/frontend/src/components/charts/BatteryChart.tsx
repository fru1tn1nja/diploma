'use client'
import Link from 'next/link'

export default function Home() {
  return (
    <div className="…">

      <Link href="http://localhost:8501" passHref>
        <a
          target="_blank"
          rel="noopener"
          className="mt-6 inline-block bg-blue-600 text-white py-2 px-4 rounded"
        >
          Check Telemetry
        </a>
      </Link>
    </div>
  )
}