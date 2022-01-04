import fetch from "node-fetch";

const PAT = "lip_T2E2rY0sd498e8DW559z";
const id = "AWLeBy0S";

const readStream = processLine => response => {
  const matcher = /\r?\n/;
  const decoder = new TextDecoder();
  let buf = '';
  return new Promise((resolve, fail) => {
    response.body.on('data', v => {
      const chunk = decoder.decode(v, { stream: true });
      buf += chunk;

      const parts = buf.split(matcher);
      buf = parts.pop();
      for (const i of parts.filter(p => p)) processLine(JSON.parse(i));
    });
    response.body.on('end', () => {
      if (buf.length > 0) processLine(JSON.parse(buf));
      resolve();
    });
    response.body.on('error', fail);
  });
};

const readFirst = processLine => response => {
  const matcher = /\r?\n/;
  const decoder = new TextDecoder();
  let buf = '';
  return new Promise((resolve, fail) => {
    response.body.on('data', v => {
      const chunk = decoder.decode(v, { stream: true });
      buf += chunk;

      const parts = buf.split(matcher);
      buf = parts.pop();
      for (const i of parts.filter(p => p)) processLine(JSON.parse(i));
      resolve();
    });
    response.body.on('end', () => {
      if (buf.length > 0) processLine(JSON.parse(buf));
      resolve();
    });
    response.body.on('error', fail);
  });
};


await fetch(`https://lichess.org/api/board/game/stream/${id}`, {
    method: "GET",
    headers: {
        "Authorization": `Bearer ${PAT}`
    }
})
.then(readFirst((line) => { console.debug(line); }))
.then(() => { console.debug("finished first element"); })
.catch((e) => { console.err(e); });

await fetch(`https://lichess.org/api/board/game/stream/${id}`, {
    method: "GET",
    headers: {
        "Authorization": `Bearer ${PAT}`
    }
})
.then(readStream((line) => {
    console.debug(JSON.stringify(line, null, 4));
    if (line.type === "gameFull") {
        console.log("populated");
    }
    else if (line.type === "gameState") {
        console.log("publishing");
    }
}))
.then(() => { console.debug("stream completed"); })
.catch((e) => { console.err(e); });
