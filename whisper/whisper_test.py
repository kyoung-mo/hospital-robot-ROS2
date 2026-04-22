import sys
import time
import whisper

# 모델로드
print("[Whisper] 모델 로딩 중...")
t0 = time.time()
model = whisper.load_model('tiny')
print(f"[Whisper] 모델 로드 완료 ({time.time() - t0:.2f}초)\n")

# 테스트할 오디오 파일
AUDIO_FILE = sys.argv[1]

# 키워드 정의
KEYWORDS = {
    'medicine': ['약', '약 주세요', '약주세요','약 갖다줘','약갖다줘'],
    'emergency': ['간호사', '도와줘', '도와주세요', '살려줘'],
    'ok':        ['괜찮아', '괜찮아요', '됐어', '됐어요'],
}

def match_keyword(text: str) -> str:
    """
    STT 결과에서 키워드 매칭
    반환값: 'medicine' / 'emergency' / 'ok' / 'unknown'
    """
    text = text.strip()
    print(f"[STT 결과] {text}")

    for category, keywords in KEYWORDS.items():
        for kw in keywords:
            if kw in text:
                return category

    return 'unknown'

def handle_result(category: str):
    """
    매칭 결과에 따른 처리
    실제 노드에서는 여기서 ROS2 토픽 발행
    """
    if category == 'medicine':
        print("[→] /hospital/medicine_request 발행 예정")
    elif category == 'emergency':
        print("[→] /hospital/emergency_call 발행 예정")
    elif category == 'ok':
        print("[→] 이상 없음. 순찰 복귀 예정")
    else:
        print("[→] 키워드 미인식. 순찰 복귀 예정")

if __name__ == '__main__':
    print(f"[Whisper] 오디오 파일 처리 중: {AUDIO_FILE}")

    t1 = time.time()
    result = model.transcribe(AUDIO_FILE, language='ko')
    stt_time = time.time() -t1
    print(f"[Time] STT 변환: {stt_time:.2f}초")

    t2 = time.time()
    text = result['text']
    category = match_keyword(text)
    match_time = time.time() -t2
    print(f"[Time] KeyWord Matching: {match_time:.4f}초")

    handle_result(category)

    print(f"\n[Time] Full Time: {stt_time + match_time:.2f}초")
