export async function post(path:string, body:object={}) {
    await fetch(`http://${process.env.NEXT_PUBLIC_GATEWAY}${path}`,{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(body)
    })
  }
  