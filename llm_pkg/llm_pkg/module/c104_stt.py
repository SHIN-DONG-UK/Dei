from faster_whisper import WhisperModel
 
class C104_Stt():
    def __init__(self):
        self.model = WhisperModel("large-v2", device="cpu", compute_type="int8")
        
    def getResult(self, file_dir):
        # STT 수행
        log_list = []
        full_text = ""
        segments, info = self.model.transcribe(file_dir, beam_size=5, language='ko', )

        for segment in segments:
            log_list.append("OUTPUT" + "[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))
            full_text += segment.text

        print(f"STT 결과: {full_text}")
        return full_text