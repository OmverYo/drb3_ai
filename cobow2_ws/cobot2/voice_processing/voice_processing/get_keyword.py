
import os
import time
import rclpy
from rclpy.node import Node

WAKEWORD_TIMEOUT = 30.0

from ament_index_python.packages import get_package_share_directory
from dotenv import load_dotenv
from openai import RateLimitError
from langchain_openai import ChatOpenAI
from langchain_core.prompts import PromptTemplate

from std_srvs.srv import Trigger

from voice_processing.wakeup_word import WakeupWord
from voice_processing.stt import STT




PACKAGE_NAME = "voice_processing"
PACKAGE_PATH = get_package_share_directory(PACKAGE_NAME)
RESOURCE_PATH = os.path.join(PACKAGE_PATH, "resource")
ENV_PATH = os.path.join(RESOURCE_PATH, ".env")
load_dotenv(dotenv_path=ENV_PATH)
openai_api_key = os.getenv("OPENAI_API_KEY")




class GetKeyword(Node):
    def __init__(self):

        print(PACKAGE_PATH, RESOURCE_PATH, ENV_PATH)

        self.llm = ChatOpenAI(
            model="gpt-4o", temperature=0.5, openai_api_key=openai_api_key
        )

        prompt_content = """
            당신은 사용자의 문장에서 특정 도구와 목적지를 추출해야 합니다.

            <목표>
            - 문장에서 다음 리스트에 포함된 도구를 최대한 정확히 추출하세요.
            - 문장에 등장하는 도구의 목적지(어디로 옮기라고 했는지)도 함께 추출하세요.

            <도구 리스트>
            - 장기말:
              cha_green, cha_red, jol_green, jol_red, ma_green, ma_red,
              po_green, po_red, sa_green, sa_red, sang_green, sang_red,
              wang_green, wang_red
            - 목적지 좌표(행, 열):
              - 행(Row): 1 ~ 10 (총 10행)
              - 열(Col): 1 ~ 9 (총 9열)

            <출력 형식>
            - 다음 형식을 반드시 따르세요: [도구1 도구2 ... / R,C R,C ...]
            - 도구가 없으면 앞쪽은 비우고, 목적지가 없으면 '/' 뒤는 비웁니다.
            - 도구와 목적지의 순서는 등장 순서를 따릅니다.

            <예시>
            - 입력: "빨간색 차를 1행 1열에 가져다 놔"  
              출력: cha_red / 1,1
            - 입력: "초록색 쫄을 5행 4열로 옮겨"  
              출력: jol_green / 5,4

            <사용자 입력>
            "{user_input}"                
        """

        self.prompt_template = PromptTemplate(
            input_variables=["user_input"], template=prompt_content
        )
        self.lang_chain = self.prompt_template | self.llm
        self.stt = STT(openai_api_key=openai_api_key)


        super().__init__("get_keyword_node")

        self.get_logger().info("MicRecorderNode initialized.")
        self.get_logger().info("wait for client's request...")
        self.get_keyword_srv = self.create_service(
            Trigger, "get_keyword", self.get_keyword
        )
        self.wakeup_word = WakeupWord()

    def extract_keyword(self, output_message):
        response = self.lang_chain.invoke({"user_input": output_message})
        result = response.content

        object, target = result.strip().split("/")

        object = object.split()
        target = target.split()

        print(f"object: {object}")
        print(f"target: {target}")
        return " ".join(object) + " / " + " ".join(target)
    
    def get_keyword(self, request, response):
        try:
            print("open stream")
            self.wakeup_word.open()
        except Exception as e:
            self.get_logger().error(f"Error: Failed to open audio stream: {e}")
            response.success = False
            response.message = ""
            return response

        detected = False
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < WAKEWORD_TIMEOUT:
                if self.wakeup_word.is_wakeup():
                    detected = True
                    break
        finally:
            self.wakeup_word.close()

        if not detected:
            self.get_logger().warn("wakeword timeout — no detection, returning failure")
            response.success = False
            response.message = ""
            return response

        # OpenAI 호출 실패를 콜백 밖으로 흘리면 노드가 통째로 죽는다.
        # 실패는 서비스 실패로 돌려주고 노드는 살려 둔다.
        try:
            output_message = self.stt.speech2text()
            keyword = self.extract_keyword(output_message)
        except RateLimitError as e:
            # 크레딧 소진은 type=insufficient_quota / code=credit_balance_exhausted 로 온다.
            # 순수 rate limit 과 달리 기다려도 안 풀리므로 안내 문구를 구분한다.
            tag = f"{getattr(e, 'type', '')} {getattr(e, 'code', '')}"
            if "insufficient_quota" in tag or "credit_balance_exhausted" in tag:
                reason = "openai_quota_exhausted"
                self.get_logger().error(
                    "OpenAI 크레딧 소진 — 충전 필요: "
                    "https://platform.openai.com/settings/organization/billing"
                )
            else:
                reason = "openai_rate_limit"
                self.get_logger().error(f"OpenAI rate limit — 잠시 후 재시도: {e}")
            response.success = False
            response.message = reason
            return response
        except Exception as e:
            self.get_logger().error(f"OpenAI 호출 실패: {type(e).__name__}: {e}")
            response.success = False
            response.message = "openai_error"
            return response

        self.get_logger().warn(f"Detected tools/targets: {keyword}")

        response.success = True
        response.message = keyword
        return response


def main():
    rclpy.init()
    node = GetKeyword()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


