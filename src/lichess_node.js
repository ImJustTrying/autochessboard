#!/usr/bin/env node

import fetch from "node-fetch"

// Credit: https://gist.github.com/ornicar/a097406810939cf7be1df8ea30e94f3e
/* FOR NODEJS
Utility function to read a ND-JSON HTTP stream.
`processLine` is a function taking a JSON object. It will be called with each element of the stream.
`response` is the result of a `fetch` request.
See usage example in the next file.
*/
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

const PAT = "lip_T2E2rY0sd498e8DW559z";
let game_id = "";
async function event_request() {
    const response = await fetch("https://lichess.org/api/stream/event", {
        method: "GET",
        headers: {
            "Authorization": `Bearer ${PAT}`
        }
    })
    .then((res) => res.text()
    .then((text) => {console.debug(text);}))
    .catch((e) => {console.debug(e);});
}

async function challenge_request() {
    const response = await fetch("https://lichess.org/api/challenge/ai", {
        method: "POST",
        headers: {
            "Content-Type": "application/x-www-form-urlencoded",
            "Authorization": `Bearer ${PAT}`
        },
        body: "level=1&days=1"
    })
    .then((res) => res.json()
    .then((json) => {
        game_id = json.id;
        console.debug(JSON.stringify(json));
    }));

    await move_request();
}

async function move_request() {
    const response = await fetch(`https://lichess.org/api/board/game/stream/${game_id}`, {
        method: "GET",
        headers: {
            "Authorization": `Bearer ${PAT}`
        }
    })
    .then((res) => {
        readStream((line) => { console.debug(JSON.stringify(line)); })(res);
    }).catch((e) => {
        console.debug("got error");
        console.debug(e);
    });
}

event_request();
challenge_request();
