import fetch from "node-fetch";

const id = "abc";
const PAT = "lip_T2E2rY0sd498e8DW559z";
const readStream = (processLine, onlyReadFirst) => response => {
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
            if (onlyReadFirst) {
                resolve();
            }
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
.then(readStream((line) => {
    if (line.error !== undefined) {
        console.error(`Error streaming game state: ${line.error}`);
    } else {
        console.debug(line);
    }
}, true))
.then(() => { console.debug("finished first element"); })
.catch((e) => { console.error(e); });

