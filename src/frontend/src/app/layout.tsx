import './globals.css'
import type { Metadata } from 'next'

export const metadata: Metadata = {
  title: 'Shared-Control Dashboard',
  description: 'Live UAV/USV monitoring',
}

/** Общий каркас Next.js (App Router) */
export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <html lang="en">
      <body className="min-h-screen bg-slate-100 text-gray-900">
        {children}
      </body>
    </html>
  )
}
